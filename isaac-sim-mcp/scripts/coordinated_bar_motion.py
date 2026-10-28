"""Move a physically grasped bar through coordinated two-arm object-space goals.

Execute inside the live simulator. Column-vector transforms, meters, radians,
seconds, USD wxyz quaternions. Object poses are measured, never commanded.
"""

import json
from pathlib import Path
import time
import traceback

import numpy as np
import omni.kit.app
import omni.physx
from omni.physx.bindings._physx import ContactEventType
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from PIL import Image
from pxr import PhysicsSchemaTools, UsdPhysics
from scipy.spatial.transform import Rotation
from isaacsim.core.prims import Articulation, RigidPrim
from isaacsim.core.utils.types import ArticulationActions

STATE = {'phase': 'idle'}
GOALS = [
    ('translate', [0, .05, .03], 0, 0, 4),
    ('yaw', [0, .05, .03], 10, 0, 4),
    ('pitch', [0, .05, .03], 10, 5, 4),
    ('return', [0, 0, 0], 0, 0, 5),
    ('final', [.025, .025, .025], -10, 0, 4),
]


async def run(output_directory, shared, checkpoints, command_dt=.05,
              slip_limit_m=.01, rotation_slip_limit_deg=3,
              origin_override=None, references_override=None, checkpoint_tag=None,
              save_checkpoints=True, sample_callback=None):
    """Test object-space trajectories, contact/slip/collision limits, and checkpoint playback.

    shared supplies verified USD-frame FK/IK and measured pad-frame helpers.
    checkpoints supplies the stage snapshot implementation. Native arm targets
    are the only motion command; the object stays dynamic under gravity.
    """
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()
    app = omni.kit.app.get_app()
    report = {'status': 'running', 'pass': False, 'input_stage': stage.GetRootLayer().identifier,
              'goals': GOALS, 'samples': [], 'segments': [], 'unexpected_contacts': [],
              'command_interval_s': command_dt,
              'limits': {'translation_slip_m': slip_limit_m, 'rotation_slip_deg': rotation_slip_limit_deg,
                         'minimum_clearance_m': .05, 'pose_error_m': .01,
                         'pose_error_deg': 5, 'bilateral_contact_fraction': .95}}
    arms, grips = [], []
    body = None
    subscription = None
    product = None
    rgb = None
    active = set()

    def rig_index(path):
        """Classify a collider's rig namespace; nonrobot shapes return None."""
        if path.startswith('/World/Arm2/'):
            return 1
        if any(path.startswith(root + '/') for root in ['/World/UR5e', '/World/RG2_Gripper', '/World/QuickChanger']):
            return 0
        return None

    def contacts(headers, data):
        """Track intended finger contact and flag inter-rig or non-mount environment contact."""
        for h in headers:
            pair = tuple(sorted([str(PhysicsSchemaTools.intToSdfPath(h.collider0)),
                                 str(PhysicsSchemaTools.intToSdfPath(h.collider1))]))
            if h.type == ContactEventType.CONTACT_LOST:
                active.discard(pair)
                continue
            active.add(pair)
            indices = [rig_index(p) for p in pair]
            unintended = False
            if all(i is not None for i in indices):
                unintended = indices[0] != indices[1]
            elif shared.OBJECT in pair:
                other = pair[1] if pair[0] == shared.OBJECT else pair[0]
                unintended = not any('/' + finger + '/' in other for finger in ['rg2_l_finger_link', 'rg2_r_finger_link'])
            elif any(i is not None for i in indices):
                robot = pair[0] if indices[0] is not None else pair[1]
                unintended = '/base_link/' not in robot
            if unintended:
                report['unexpected_contacts'].append({'time_s': timeline.get_current_time(),
                                                     'phase': STATE['phase'], 'pair': pair})

    async def settle(seconds):
        """Wait on simulation time with finite-state, collision, and stall checks."""
        start, wall = timeline.get_current_time(), time.monotonic()
        while timeline.get_current_time() - start < seconds - 1e-9:
            await app.next_update_async()
            if not timeline.is_playing() or time.monotonic() - wall > 30:
                raise RuntimeError('Simulation stopped or stalled')
            if any(not np.all(np.isfinite(a.get_joint_positions())) for a in arms + grips):
                timeline.stop()
                raise RuntimeError('Non-finite articulation state')
            if report['unexpected_contacts']:
                raise RuntimeError('Unexpected collision detected; trajectory advancement stopped')
            if sample_callback is not None:
                sample_callback(stage, timeline.get_current_time())

    def object_pose():
        """Return the measured dynamic object's column-vector transform."""
        position, quaternion = body.get_world_poses()
        q = quaternion[0]
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        transform[:3, 3] = position[0]
        if not np.all(np.isfinite(transform)):
            raise RuntimeError('Non-finite object pose')
        return transform

    def measure(goal, references):
        """Measure object tracking, pad-frame slip, clearance, and four-finger contact."""
        actual = object_pose()
        slips, angles = [], []
        for root, reference in zip(shared.GRIPS, references):
            relative = np.linalg.inv(shared.grasp_frame(stage, root)) @ actual
            slips.append(float(np.linalg.norm(relative[:3, 3] - reference[:3, 3])))
            angles.append(float(np.degrees(np.linalg.norm(Rotation.from_matrix(
                relative[:3, :3] @ reference[:3, :3].T).as_rotvec()))))
        flags = [[any(root + '/' + finger + '/' in str(p) for p in active)
                  for finger in ['rg2_l_finger_link', 'rg2_r_finger_link']] for root in shared.GRIPS]
        return {'time_s': timeline.get_current_time(), 'phase': STATE['phase'],
                'object_position_m': actual[:3, 3].tolist(), 'object_rotation_matrix': actual[:3, :3].tolist(),
                'goal_position_m': goal[:3, 3].tolist(), 'goal_rotation_matrix': goal[:3, :3].tolist(),
                'translation_slip_m': slips, 'rotation_slip_deg': angles,
                'bottom_z_m': float(actual[2, 3] - np.abs(actual[2, :3]) @ np.array([.325, .0175, .03])),
                'pose_error_m': float(np.linalg.norm(actual[:3, 3] - goal[:3, 3])),
                'pose_error_deg': float(np.degrees(np.linalg.norm(Rotation.from_matrix(
                    goal[:3, :3] @ actual[:3, :3].T).as_rotvec()))), 'finger_contacts': flags}

    def check(measurement):
        """Fail immediately on geometric slip, support clearance, or pose error limits."""
        if (max(measurement['translation_slip_m']) > slip_limit_m
            or max(measurement['rotation_slip_deg']) > rotation_slip_limit_deg
            or measurement['bottom_z_m'] < .05
            or measurement['pose_error_m'] > .01 or measurement['pose_error_deg'] > 5):
            raise RuntimeError('Slip, clearance, or object tracking limit exceeded')

    try:
        STATE['phase'] = 'initialize grasp'
        if not timeline.is_playing():
            raise RuntimeError('Load and play the accepted shared-grasp checkpoint first')
        for i, (ap, gp) in enumerate(zip(shared.ARMS, shared.GRIPS)):
            arm = Articulation(ap, name=f'coordinated_arm_{i}')
            arm.initialize()
            grip = Articulation(gp, name=f'coordinated_grip_{i}')
            grip.initialize()
            index = grip.dof_names.index('rg2_gripper_joint')
            UsdPhysics.DriveAPI(stage.GetPrimAtPath(gp + '/joints/rg2_gripper_joint'), 'angular').GetTargetPositionAttr().Set(float(np.degrees(-.45)))
            grip.apply_action(ArticulationActions(joint_positions=np.array([[-.45]]), joint_indices=np.array([index])))
            arms.append(arm)
            grips.append(grip)
        body = RigidPrim(shared.OBJECT, name='coordinated_bar')
        body.initialize()
        subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contacts)
        await settle(.5)
        origin = object_pose() if origin_override is None else np.array(origin_override)
        grasp_relative = [np.linalg.inv(origin) @ shared.grasp_frame(stage, gp) for gp in shared.GRIPS]
        references = ([np.linalg.inv(shared.grasp_frame(stage, gp)) @ origin for gp in shared.GRIPS]
                      if references_override is None else [np.array(r) for r in references_override])
        models = [shared.Kinematics(stage, ap, gp) for ap, gp in zip(shared.ARMS, shared.GRIPS)]
        seeds = [a.get_joint_positions()[0].copy() for a in arms]
        report['initial'] = measure(origin, references)
        check(report['initial'])
        product = rep.create.render_product('/World/dual_arm_overview', (960, 720),
                                             force_new=True, name='RISE_CoordinatedBar')
        rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb.attach([product.path])
        previous = origin.copy()
        bad_contact_streak = 0
        for name, offset, yaw, pitch, duration in GOALS:
            STATE['phase'] = name
            goal = origin.copy()
            goal[:3, 3] += offset
            goal[:3, :3] = (Rotation.from_euler('z', yaw, degrees=True).as_matrix()
                            @ Rotation.from_euler('y', pitch, degrees=True).as_matrix() @ origin[:3, :3])
            rotation_delta = Rotation.from_matrix(goal[:3, :3] @ previous[:3, :3].T).as_rotvec()
            count = max(2, int(np.ceil(duration / command_dt)))
            segment_start = len(report['samples'])
            for step in range(1, count + 1):
                alpha = step / count
                blend = alpha * alpha * (3 - 2 * alpha)
                target = np.eye(4)
                target[:3, 3] = previous[:3, 3] + blend * (goal[:3, 3] - previous[:3, 3])
                target[:3, :3] = Rotation.from_rotvec(blend * rotation_delta).as_matrix() @ previous[:3, :3]
                for i, (model, relative, arm) in enumerate(zip(models, grasp_relative, arms)):
                    grasp = target @ relative
                    seeds[i] = model.solve(grasp[:3, 3], grasp[:3, :3], seeds[i])
                    arm.apply_action(ArticulationActions(joint_positions=np.array([seeds[i]])))
                await settle(command_dt)
                measurement = measure(target, references)
                report['samples'].append(measurement)
                check(measurement)
                bad_contact_streak = (0 if all(all(pair) for pair in measurement['finger_contacts'])
                                      else bad_contact_streak + 1)
                if bad_contact_streak > 4:
                    raise RuntimeError('Bilateral finger contact lost for more than four samples')
                STATE['samples'] = len(report['samples'])
            await settle(.5)
            measurement = measure(goal, references)
            report['samples'].append(measurement)
            check(measurement)
            report['segments'].append({'name': name, 'end': measurement,
                                       'sample_count': len(report['samples']) - segment_start})
            # Standalone Isaac needs an explicit render capture before readback.
            await rep.orchestrator.step_async(rt_subframes=4, pause_timeline=False)
            Image.fromarray(rgb.get_data()).save(str(output / f'{name}.png'))
            previous = goal
        STATE['phase'] = 'final hold'
        for _ in range(24):
            await settle(1 / 12)
            measurement = measure(previous, references)
            report['samples'].append(measurement)
            check(measurement)
        measurements = report['samples']
        report['summary'] = {
            'sample_count': len(measurements),
            'simulation_duration_s': measurements[-1]['time_s'] - measurements[0]['time_s'],
            'max_translation_slip_m': max(max(x['translation_slip_m']) for x in measurements),
            'max_rotation_slip_deg': max(max(x['rotation_slip_deg']) for x in measurements),
            'minimum_clearance_m': min(x['bottom_z_m'] for x in measurements),
            'max_object_pose_error_m': max(x['pose_error_m'] for x in measurements),
            'max_object_rotation_error_deg': max(x['pose_error_deg'] for x in measurements),
            'bilateral_contact_fraction': float(np.mean([all(all(p) for p in x['finger_contacts']) for x in measurements])),
            'unexpected_collision_count': len(report['unexpected_contacts'])}
        if report['summary']['bilateral_contact_fraction'] < .95:
            raise RuntimeError('Bilateral contact fraction below 95 percent')
        if not save_checkpoints:
            report['status'] = 'complete'
            report['pass'] = True
            return report
        # Persist current arm targets as well as the force-limited gripper close
        # commands; a queued historical USD goal must not overwrite tensor motion.
        for ap, arm in zip(shared.ARMS, arms):
            q = arm.get_joint_positions()[0]
            for name, angle in zip(shared.JOINTS, q):
                joint = stage.GetPrimAtPath(ap + '/joints/' + name)
                UsdPhysics.DriveAPI(joint, 'angular').GetTargetPositionAttr().Set(float(np.degrees(angle)))
        await settle(.1)
        report['checkpoint'] = checkpoints.save_checkpoint(stage,
            '07-coordinated-bar-motion-pending-replay' if checkpoint_tag is None else checkpoint_tag + '-pending-replay',
            'Translation, yaw, pitch, return and final-pose slip/collision checks passed; exact-file replay pending.',
            keep_gripper_targets=True)
        rgb.detach([product.path])
        product.destroy()
        rgb = product = None
        STATE['phase'] = 'checkpoint replay'
        active.clear()
        arms.clear()
        grips.clear()
        timeline.stop()
        await omni.usd.get_context().open_stage_async(report['checkpoint'])
        stage = omni.usd.get_context().get_stage()
        timeline.play()
        await settle(.25)
        for i, (ap, gp) in enumerate(zip(shared.ARMS, shared.GRIPS)):
            arm = Articulation(ap, name=f'replayed_coordinated_arm_{i}')
            arm.initialize()
            grip = Articulation(gp, name=f'replayed_coordinated_grip_{i}')
            grip.initialize()
            arms.append(arm)
            grips.append(grip)
        body = RigidPrim(shared.OBJECT, name='replayed_coordinated_bar')
        body.initialize()
        await settle(2)
        report['replayed'] = measure(previous, references)
        check(report['replayed'])
        if not all(all(pair) for pair in report['replayed']['finger_contacts']):
            raise RuntimeError('Checkpoint replay lost a fingertip contact')
        report['status'] = 'complete'
        report['pass'] = True
        report['validated_checkpoint'] = checkpoints.save_checkpoint(stage,
            '08-coordinated-bar-motion-validated' if checkpoint_tag is None else checkpoint_tag + '-validated',
            'Coordinated translation/yaw/pitch/return passed slip/collision bounds; held pose passed physical checkpoint replay.',
            keep_gripper_targets=True)
    except Exception:
        report['status'] = 'failed'
        report['error'] = traceback.format_exc()
        for arm in arms:
            q = arm.get_joint_positions()
            if np.all(np.isfinite(q)):
                arm.apply_action(ArticulationActions(joint_positions=q))
    finally:
        subscription = None
        if rgb is not None:
            rgb.detach([product.path])
        if product is not None:
            product.destroy()
        (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        STATE['phase'] = 'finished'
    return report
