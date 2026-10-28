"""Bimanual shared-bar grasp planning and physical lift validation in Isaac Sim.

Run only inside the live simulator. Distances: meters, angles: radians; numpy
transforms use column vectors and quaternions enter scipy as xyzw. No physical
driver, grasp weld, disabled object gravity, or object teleport during lift.
"""

import json
from pathlib import Path
import time
import traceback

import numpy as np
import omni.kit.app
import omni.physx
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from PIL import Image
from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics, UsdShade
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from isaacsim.core.prims import Articulation, RigidPrim
from isaacsim.core.utils.types import ArticulationActions

ARMS = ['/World/UR5e', '/World/Arm2/UR5e']
GRIPS = ['/World/RG2_Gripper', '/World/Arm2/RG2_Gripper']
OBJECT = '/World/SharedBar'
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
STATE = {'phase': 'idle'}


def matrix(stage, path):
    """Return a USD prim's column-vector world transform."""
    return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T


def joint_frame(position, quaternion):
    """Convert a USD joint's local translation and wxyz quaternion to a matrix."""
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([*quaternion.GetImaginary(), quaternion.GetReal()]).as_matrix()
    result[:3, 3] = position
    return result


def grasp_frame(stage, root):
    """Compute the grasp frame from measured inner-pad datums and the hand origin."""
    points = []
    for name, local in [('rg2_l_finger_link', [.0379, -.01898, 0, 1]),
                        ('rg2_r_finger_link', [.0379, .01898, 0, 1])]:
        points.append((matrix(stage, root + '/' + name) @ np.array(local))[:3])
    center = (points[0] + points[1]) / 2
    closing = points[1] - points[0]
    closing /= np.linalg.norm(closing)
    approach = center - matrix(stage, root + '/rg2_hand_link')[:3, 3]
    approach -= closing * np.dot(approach, closing)
    approach /= np.linalg.norm(approach)
    result = np.eye(4)
    result[:3, :3] = np.column_stack([approach, closing, np.cross(approach, closing)])
    result[:3, 3] = center
    return result


class Kinematics:
    """Six-joint FK from USD body frames, with numerically solved grasp targets."""

    def __init__(self, stage, arm_path, gripper_path):
        self.base = matrix(stage, arm_path + '/base_link')
        self.frames = []
        for name in JOINTS:
            prim = stage.GetPrimAtPath(arm_path + '/joints/' + name)
            joint = UsdPhysics.Joint(prim)
            self.frames.append((joint_frame(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get()),
                                np.linalg.inv(joint_frame(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get())),
                                str(prim.GetAttribute('physics:axis').Get())))
        self.tool = np.linalg.inv(matrix(stage, arm_path + '/wrist_3_link')) @ grasp_frame(stage, gripper_path)

    def forward(self, q):
        """Return the grasp transform for a DOF-ordered radians vector."""
        current = self.base.copy()
        for angle, (parent, child_inverse, axis) in zip(q, self.frames):
            turn = np.eye(4)
            turn[:3, :3] = Rotation.from_euler(axis.lower(), angle).as_matrix()
            current = current @ parent @ turn @ child_inverse
        return current @ self.tool

    def solve(self, position, orientation, seed, multi=False):
        """Solve pose IK and reject unreachable targets rather than accepting residual error."""
        def residual(q):
            actual = self.forward(q)
            return np.r_[actual[:3, 3] - position,
                         .15 * Rotation.from_matrix(orientation @ actual[:3, :3].T).as_rotvec()]
        candidates = [np.array(seed)]
        if multi:
            for elbow in [-1.7, 1.7]:
                for wrist in [-1.57, 1.57]:
                    candidates.append(np.array([0, -1.3, elbow, -elbow, wrist, 0]))
            rng = np.random.default_rng(7)
            candidates.extend(rng.uniform(-2, 2, (10, 6)))
        solutions = []
        lower = np.array([-6.28, -6.28, -3.14, -6.28, -6.28, -6.28])
        upper = -lower
        for start in candidates:
            solution = least_squares(residual, np.clip(start, lower, upper), bounds=(lower, upper), max_nfev=400,
                                     ftol=1e-9, xtol=1e-9, gtol=1e-9)
            if np.linalg.norm(residual(solution.x)) < 1e-4:
                solutions.append(solution.x)
        if not solutions:
            raise RuntimeError(f'Unreachable grasp target {position}')
        return min(solutions, key=lambda q: np.linalg.norm(q - seed))


async def inspect_and_plan(output_directory):
    """Open both grippers, verify FK against the live stage, and plan top-down bar grasps."""
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    stage = omni.usd.get_context().get_stage()
    arms, grips = [], []
    for index, (arm_path, grip_path) in enumerate(zip(ARMS, GRIPS)):
        arm = Articulation(arm_path, name=f'grasp_arm_{index}')
        arm.initialize()
        grip = Articulation(grip_path, name=f'grasp_gripper_{index}')
        grip.initialize()
        actuated = grip.dof_names.index('rg2_gripper_joint')
        grip.apply_action(ArticulationActions(joint_positions=np.array([[.78]]), joint_indices=np.array([actuated])))
        arms.append(arm)
        grips.append(grip)
    for _ in range(150):
        await omni.kit.app.get_app().next_update_async()
        if any(not np.all(np.isfinite(a.get_joint_positions())) for a in arms + grips):
            omni.timeline.get_timeline_interface().stop()
            raise RuntimeError('Nonfinite arm/gripper state while opening')
    report = {'fk': [], 'plans': []}
    for index, (arm_path, grip_path, arm) in enumerate(zip(ARMS, GRIPS, arms)):
        model = Kinematics(stage, arm_path, grip_path)
        seed = arm.get_joint_positions()[0]
        predicted = model.forward(seed)
        measured = grasp_frame(stage, grip_path)
        error = float(np.linalg.norm(predicted[:3, 3] - measured[:3, 3]))
        rotation_error = float(np.linalg.norm(Rotation.from_matrix(predicted[:3, :3] @ measured[:3, :3].T).as_rotvec()))
        report['fk'].append({'position_error_m': error, 'rotation_error_rad': rotation_error,
                              'grasp_pose': measured.tolist()})
        if error > .001 or rotation_error > .01:
            raise RuntimeError(f'FK verification failed for arm {index + 1}: {error}, {rotation_error}')
        # Approach downward, close across the bar's 35 mm world-Y width.
        approach = np.array([0, 0, -1])
        closing = np.array([0, 1 if index == 0 else -1, 0])
        orientation = np.column_stack([approach, closing, np.cross(approach, closing)])
        x = .22 if index == 0 else -.22
        pre = model.solve(np.array([x, -.3, .30]), orientation, seed, multi=True)
        # Keep truss-link collision geometry above the 60 mm bar; only the
        # lower finger-pad region should engage its side surfaces.
        grasp = model.solve(np.array([x, -.3, .085]), orientation, pre, multi=False)
        lifted = model.solve(np.array([x, -.3, .185]), orientation, grasp, multi=False)
        report['plans'].append({'pre_rad': pre.tolist(), 'grasp_rad': grasp.tolist(),
                                'lift_rad': lifted.tolist(), 'grasp_x_m': x,
                                'orientation': orientation.tolist()})
    (output / 'planning.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    STATE['planning'] = report
    STATE['arms'] = arms
    STATE['grips'] = grips
    STATE['phase'] = 'planned'
    return report


async def run(output_directory, checkpoint_support):
    """Attempt a real two-gripper friction grasp, 100 mm lift, hold, release, and checkpoint reload.

    Success requires both fingers of both grippers in object contact, 50 mm
    minimum table clearance during a two-second hold, less than 10 mm slip in
    either grasp frame, and an object drop when both fingers open. A checkpoint
    is accepted only after it also reloads and sustains the grasp physically.
    """
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    planning = json.loads((output.parent / 'planning.json').read_text())
    plans = planning['plans']
    report = {'status': 'running', 'pass': False, 'object': {'size_m': [.65, .035, .06],
              'mass_kg': .1, 'static_friction': .8, 'dynamic_friction': .6, 'gravity': True},
              'samples': [], 'contact_events': [], 'planning': planning}
    timeline = omni.timeline.get_timeline_interface()
    app = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()
    arms, grips = [], []
    active = set()
    subscription = None
    render_product = None
    annotator = None
    body = None

    async def settle(seconds):
        """Advance live simulation time while checking finite articulated and object states."""
        start, wall = timeline.get_current_time(), time.monotonic()
        while timeline.get_current_time() - start < seconds:
            await app.next_update_async()
            if not timeline.is_playing() or time.monotonic() - wall > 30:
                raise RuntimeError('Simulation stopped or stalled')
            if any(not np.all(np.isfinite(a.get_joint_positions())) for a in arms + grips):
                timeline.stop()
                raise RuntimeError('Non-finite articulated state')
            if body is not None and not np.all(np.isfinite(body.get_world_poses()[0])):
                timeline.stop()
                raise RuntimeError('Non-finite object state')

    def command_grippers(theta):
        """Command the actuated joint only; mimic joints follow physically."""
        for root, grip in zip(GRIPS, grips):
            # Keep USD and tensor targets consistent: a queued USD drive update
            # can otherwise overwrite a later tensor-only release command.
            joint = stage.GetPrimAtPath(root + '/joints/rg2_gripper_joint')
            UsdPhysics.DriveAPI(joint, 'angular').GetTargetPositionAttr().Set(float(np.degrees(theta)))
            index = grip.dof_names.index('rg2_gripper_joint')
            grip.apply_action(ArticulationActions(joint_positions=np.array([[theta]]),
                                                 joint_indices=np.array([index])))

    async def move(targets, duration):
        """Interpolate both arm targets with a smooth joint-space trajectory."""
        initial = [a.get_joint_positions()[0].copy() for a in arms]
        count = int(np.ceil(duration / .025))
        for k in range(1, count + 1):
            t = k / count
            blend = t * t * (3 - 2 * t)
            for arm, q0, q1 in zip(arms, initial, targets):
                arm.apply_action(ArticulationActions(joint_positions=np.array([q0 + blend * (q1 - q0)])))
            await settle(.025)

    def contact_callback(headers, data):
        """Track actual bar contacts, distinguishing both fingers of each gripper and the table."""
        from omni.physx.bindings._physx import ContactEventType
        for h in headers:
            pair = tuple(sorted([str(PhysicsSchemaTools.intToSdfPath(h.collider0)),
                                 str(PhysicsSchemaTools.intToSdfPath(h.collider1))]))
            if OBJECT not in pair:
                continue
            if h.type == ContactEventType.CONTACT_LOST:
                active.discard(pair)
            else:
                active.add(pair)
                report['contact_events'].append({'phase': STATE['phase'], 'time_s': timeline.get_current_time(),
                                                'pair': pair})

    def object_transform():
        """Construct the live object's world transform from its tensor pose."""
        position, orientation = body.get_world_poses()
        q = orientation[0]
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        transform[:3, 3] = position[0]
        return transform

    def sample():
        """Measure support clearance and independent left/right contacts for both grippers."""
        transform = object_transform()
        flags = [[any(root + '/' + finger + '/' in str(pair) for pair in active)
                  for finger in ['rg2_l_finger_link', 'rg2_r_finger_link']] for root in GRIPS]
        return {'time_s': timeline.get_current_time(), 'position_m': transform[:3, 3].tolist(),
                'bottom_z_m': float(transform[2, 3] - np.abs(transform[2, :3]) @ np.array([.325, .0175, .03])),
                'finger_contacts': flags, 'active_contacts': sorted(active)}

    try:
        STATE['phase'] = 'clear setup area'
        for index, (ap, gp) in enumerate(zip(ARMS, GRIPS)):
            arm = Articulation(ap, name=f'prepare_shared_arm_{index}')
            arm.initialize()
            grip = Articulation(gp, name=f'prepare_shared_grip_{index}')
            grip.initialize()
            arms.append(arm)
            grips.append(grip)
        command_grippers(.78)
        await settle(1)
        await move([np.array(p['pre_rad']) for p in plans], 3)
        await settle(.5)
        arms.clear()
        grips.clear()
        STATE['phase'] = 'shared object setup'
        timeline.pause()
        if stage.GetPrimAtPath(OBJECT):
            stage.RemovePrim(OBJECT)
        cube = UsdGeom.Cube.Define(stage, OBJECT)
        cube.CreateSizeAttr(1)
        cube.ClearXformOpOrder()
        cube.AddTranslateOp().Set(Gf.Vec3d(0, -.3, .031))
        cube.AddScaleOp().Set(Gf.Vec3f(.65, .035, .06))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.8, .22, .08)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(cube.GetPrim())
        mass.CreateMassAttr(.1)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*(.1 / 12 * np.array([.035**2 + .06**2,
                                                             .65**2 + .06**2, .65**2 + .035**2]))))
        PhysxSchema.PhysxContactReportAPI.Apply(cube.GetPrim()).CreateThresholdAttr(0)
        PhysxSchema.PhysxRigidBodyAPI.Apply(cube.GetPrim()).CreateDisableGravityAttr(False)
        material = UsdShade.Material.Define(stage, '/World/SharedBarMaterial')
        pm = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        pm.CreateStaticFrictionAttr(.8)
        pm.CreateDynamicFrictionAttr(.6)
        pm.CreateRestitutionAttr(0)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material, materialPurpose='physics')
        for root in GRIPS:
            joint = stage.GetPrimAtPath(root + '/joints/rg2_gripper_joint')
            UsdPhysics.DriveAPI(joint, 'angular').GetTargetPositionAttr().Set(float(np.degrees(.78)))
        for root in ARMS:
            for name in JOINTS:
                joint = stage.GetPrimAtPath(root + '/joints/' + name)
                UsdPhysics.DriveAPI(joint, 'angular').GetTargetPositionAttr().Set(
                    joint.GetAttribute('state:angular:physics:position').Get())
        setup_path = output / 'setup.usda'
        checkpoint_support.save_scene_overlay(stage, setup_path)
        timeline.stop()
        await omni.usd.get_context().open_stage_async(str(setup_path))
        stage = omni.usd.get_context().get_stage()
        subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact_callback)
        timeline.play()
        await settle(.5)
        for index, (ap, gp) in enumerate(zip(ARMS, GRIPS)):
            arm = Articulation(ap, name=f'shared_arm_{index}')
            arm.initialize()
            grip = Articulation(gp, name=f'shared_grip_{index}')
            grip.initialize()
            arms.append(arm)
            grips.append(grip)
        body = RigidPrim(OBJECT, name='shared_bar')
        body.initialize()
        command_grippers(.78)
        await settle(1)
        render_product = rep.create.render_product('/World/dual_arm_overview', (960, 720),
                                                    force_new=True, name='RISE_SharedObjectValidation')
        annotator = rep.AnnotatorRegistry.get_annotator('rgb')
        annotator.attach([render_product.path])
        STATE['phase'] = 'pregrasp'
        await move([np.array(p['pre_rad']) for p in plans], 4)
        await settle(1)
        report['pregrasp'] = sample()
        Image.fromarray(annotator.get_data()).save(str(output / 'pregrasp.png'))
        STATE['phase'] = 'approach'
        await move([np.array(p['grasp_rad']) for p in plans], 3)
        await settle(.5)
        STATE['phase'] = 'closing'
        command_grippers(-.45)
        await settle(2)
        report['closed'] = sample()
        report['gripper_angles_closed_rad'] = [g.get_joint_positions().tolist() for g in grips]
        Image.fromarray(annotator.get_data()).save(str(output / 'closed.png'))
        if not all(all(pair) for pair in report['closed']['finger_contacts']):
            raise RuntimeError('Both fingers of both grippers did not contact the shared object')
        references = [np.linalg.inv(grasp_frame(stage, root)) @ object_transform() for root in GRIPS]
        STATE['phase'] = 'lift'
        await move([np.array(p['lift_rad']) for p in plans], 3)
        STATE['phase'] = 'hold'
        for _ in range(24):
            await settle(1 / 12)
            measurement = sample()
            measurement['relative_slip_m'] = [float(np.linalg.norm((np.linalg.inv(grasp_frame(stage, root)) @ object_transform())[:3, 3] - ref[:3, 3]))
                                               for root, ref in zip(GRIPS, references)]
            report['samples'].append(measurement)
        report['minimum_hold_clearance_m'] = min(s['bottom_z_m'] for s in report['samples'])
        report['maximum_slip_m'] = max(max(s['relative_slip_m']) for s in report['samples'])
        report['both_grippers_contact_fraction'] = float(np.mean([all(all(p) for p in s['finger_contacts']) for s in report['samples']]))
        report['hold_pass'] = (report['minimum_hold_clearance_m'] >= .05 and report['maximum_slip_m'] <= .01
                               and report['both_grippers_contact_fraction'] >= .8)
        Image.fromarray(annotator.get_data()).save(str(output / 'lift.png'))
        if not report['hold_pass']:
            raise RuntimeError('Shared lift failed clearance, slip, or bilateral contact acceptance')
        for root in GRIPS:
            UsdPhysics.DriveAPI(stage.GetPrimAtPath(root + '/joints/rg2_gripper_joint'), 'angular').GetTargetPositionAttr().Set(float(np.degrees(-.45)))
        held_checkpoint = checkpoint_support.save_checkpoint(stage, '05-shared-lift-pending-release-check',
            'Shared object lift/hold passed; physical release and checkpoint playback still pending.', keep_gripper_targets=True)
        report['held_checkpoint'] = held_checkpoint
        held_z = float(object_transform()[2, 3])
        STATE['phase'] = 'release'
        command_grippers(.78)
        await settle(1.5)
        report['release'] = sample()
        report['release_drop_m'] = held_z - report['release']['position_m'][2]
        if report['release_drop_m'] < .05:
            raise RuntimeError('Release did not produce an independent gravity-driven drop')
        annotator.detach([render_product.path])
        render_product.destroy()
        annotator = render_product = None
        STATE['phase'] = 'checkpoint reload'
        arms.clear()
        grips.clear()
        body = None
        active.clear()
        timeline.stop()
        await omni.usd.get_context().open_stage_async(held_checkpoint)
        stage = omni.usd.get_context().get_stage()
        timeline.play()
        await settle(.5)
        for index, (ap, gp) in enumerate(zip(ARMS, GRIPS)):
            arm = Articulation(ap, name=f'reloaded_shared_arm_{index}')
            arm.initialize()
            grip = Articulation(gp, name=f'reloaded_shared_grip_{index}')
            grip.initialize()
            arms.append(arm)
            grips.append(grip)
        body = RigidPrim(OBJECT, name='reloaded_shared_bar')
        body.initialize()
        await settle(2)
        report['reloaded'] = sample()
        report['pass'] = (report['reloaded']['bottom_z_m'] >= .05
                          and all(all(pair) for pair in report['reloaded']['finger_contacts']))
        if not report['pass']:
            raise RuntimeError('Grasp checkpoint did not sustain its physical hold after reload')
        report['validated_checkpoint'] = checkpoint_support.save_checkpoint(stage, '06-shared-grasp-lift-validated',
            'Both grippers held/lifted shared bar, gravity release passed, checkpoint playback retained grasp.', keep_gripper_targets=True)
        report['status'] = 'complete'
    except Exception:
        report['status'] = 'failed'
        report['error'] = traceback.format_exc()
    finally:
        subscription = None
        if annotator is not None:
            annotator.detach([render_product.path])
        if render_product is not None:
            render_product.destroy()
        (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        STATE['phase'] = 'finished'
    return report
