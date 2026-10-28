"""Create and validate two lab UR5e/QC/RG2 rigs in the running Isaac Sim.

Import this file through MCP and schedule run(output_directory) on Kit's event
loop. Units are meters, radians, seconds, and contact impulse in N*s. USD matrix
composition uses row vectors; USD quaternions are wxyz. No robot driver is used.
"""

import json
from pathlib import Path
import time
import traceback
from datetime import datetime, timezone
import hashlib

import numpy as np
import omni.kit.app
import omni.physx
from omni.physx.bindings._physx import ContactEventType
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from PIL import Image
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationActions

RIG_ROOTS = ('/World/UR5e', '/World/QuickChanger', '/World/RG2_Gripper')
SECOND_ROOT = '/World/Arm2'
FIRST_BASE_TRANSLATION_M = (0.75, -0.30, 0.0)
BASE_TRANSLATION_M = (-0.75, -0.30, 0.0)
HOME_RAD = (0.0, -0.80, 0.0, 0.0, 0.0, 0.0)
LAYOUT = 'opposite-table-ends-v1'
CHECKPOINT_DIRECTORY = Path(__file__).resolve().parents[2] / 'checkpoints'
STATE = {'phase': 'idle'}


def create_overview_camera(stage):
    """Create a separate overview that frames both rigs without moving the existing camera."""
    camera = UsdGeom.Camera.Define(stage, '/World/dual_arm_overview')
    camera.CreateFocalLengthAttr(35)
    camera.CreateHorizontalApertureAttr(36)
    camera.CreateVerticalApertureAttr(27)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 100))
    matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(2.7, -3.4, 2.3),
                                    Gf.Vec3d(0, -.325, .25), Gf.Vec3d(0, 0, 1)).GetInverse()
    camera.MakeMatrixXform().Set(matrix)
    return str(camera.GetPath())


def save_scene_overlay(stage, destination):
    """Export authored opinions with explicit relative asset paths anchored to their source."""
    original = stage.GetRootLayer()
    snapshot = Sdf.Layer.CreateAnonymous('dual_arm_scene.usda')
    snapshot.ImportFromString(original.ExportToString())
    UsdUtils.ModifyAssetPaths(snapshot, lambda path: Sdf.ComputeAssetPathRelativeToLayer(original, path)
                             if path.startswith(('./', '../')) else path)
    if not snapshot.Export(str(destination)):
        raise RuntimeError('Could not export dual-arm scene overlay')


def save_checkpoint(stage, label, note, keep_gripper_targets=False):
    """Save a stationary USDA snapshot, JSON provenance, and SHA256 without changing the live scene.

    Captures current PhysX-written link poses and angular states together. On the
    snapshot only, zero velocities and hold each driven joint at its current
    position so reopening does not replay a previous trajectory. Source assets
    remain referenced at anchored paths; this is a local checkpoint, not a
    portable asset bundle or serialized PhysX solver state.

    keep_gripper_targets preserves force-limited RG2 closing targets for a
    loaded grasp checkpoint; freezing those targets at contact would remove
    the commanded preload. Velocities are still zeroed in the saved layer.
    """
    CHECKPOINT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    path = CHECKPOINT_DIRECTORY / f'{stamp}-{label}.usda'
    save_scene_overlay(stage, path)
    # Robot joint types come from payloads: LoadNone would skip normalization
    # because the authored override prims alone are untyped.
    saved = Usd.Stage.Open(str(path), load=Usd.Stage.LoadAll)
    normalized_joints = 0
    for prim in saved.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            position = prim.GetAttribute('state:angular:physics:position').Get()
            if position is not None:
                prim.GetAttribute('state:angular:physics:velocity').Set(0.0)
                drive = UsdPhysics.DriveAPI(prim, 'angular')
                if prim.HasAPI(UsdPhysics.DriveAPI, 'angular'):
                    if not (keep_gripper_targets and prim.GetName() == 'rg2_gripper_joint'):
                        drive.GetTargetPositionAttr().Set(position)
                    drive.GetTargetVelocityAttr().Set(0.0)
                normalized_joints += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body = UsdPhysics.RigidBodyAPI(prim)
            body.GetVelocityAttr().Set(Gf.Vec3f(0))
            body.GetAngularVelocityAttr().Set(Gf.Vec3f(0))
    metadata = {'created_utc': stamp, 'label': label, 'note': note,
                'source_stage': stage.GetRootLayer().identifier,
                'composition_errors': sorted({str(e) for prim in saved.Traverse()
                                               for e in prim.GetPrimIndex().localErrors}),
                'arm1_present': bool(saved.GetPrimAtPath('/World/UR5e')),
                'arm2_present': bool(saved.GetPrimAtPath('/World/Arm2/UR5e')),
                'normalized_joint_count': normalized_joints,
                'scope': 'local referenced USD stage, stationary joint targets; no solver-state serialization'}
    saved.GetRootLayer().customLayerData = {'riseCheckpoint': json.dumps(metadata)}
    saved.GetRootLayer().Save()
    metadata['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    path.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    if metadata['composition_errors']:
        raise RuntimeError(f'Checkpoint composition errors: {metadata["composition_errors"]}')
    return str(path)


def restore_second_rig_joint_frames(stage):
    """Restore canonical first-rig joint frames after posed relationship remapping.

    Call while physics is stopped and keep complete arm/gripper poses together.
    This also repairs historical checkpoints whose joint frames were recomputed
    by PhysX authoring when copied joint body relationships were assigned.
    """
    count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(SECOND_ROOT)):
        if prim.IsA(UsdPhysics.Joint):
            original = stage.GetPrimAtPath(str(prim.GetPath()).replace(SECOND_ROOT + '/', '/World/', 1))
            if original:
                for name in ('physics:localPos0', 'physics:localPos1', 'physics:localRot0', 'physics:localRot1'):
                    value = original.GetAttribute(name).Get()
                    if value is not None:
                        prim.GetAttribute(name).Set(value)
                count += 1
    return count


async def run(output_directory):
    """Clone the complete lab rig, test independent movement and contact, and save evidence.

    Bases are at opposite table ends, x=+/-0.75 m and y=-0.30 m, facing inward.
    Relationship targets are remapped
    across all three cloned roots, including both gripper welds and materials.
    Contact approach stops at the first inter-rig event, followed by withdrawal.
    Leaves both rigs playing at their separated home targets. Repeated runs reuse
    the second rig instead of duplicating it. Source USD assets are not modified.
    """
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'movement': [], 'contact_events': [], 'checkpoints': [],
              'configuration': {'arm1': '/World/UR5e', 'arm2': '/World/Arm2/UR5e',
                                'first_base_translation_m': FIRST_BASE_TRANSLATION_M,
                                'second_base_translation_m': BASE_TRANSLATION_M,
                                'second_relative_yaw_deg': 180, 'home_rad': HOME_RAD,
                                'gravity': 'disabled on both complete robot chains',
                                'control': 'independent simulator articulation targets; no second ROS graph'}}
    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()
    app = omni.kit.app.get_app()
    active_contacts = set()
    subscription = None
    arms = []
    home = np.array([HOME_RAD], dtype=np.float32)
    render_product = None
    rgb = None

    async def settle(seconds):
        """Wait in simulation time, aborting promptly on stopped or stalled physics."""
        start = timeline.get_current_time()
        wall_start = time.monotonic()
        while timeline.get_current_time() - start < seconds:
            await app.next_update_async()
            if not timeline.is_playing() or time.monotonic() - wall_start > 30:
                raise RuntimeError('Simulation stopped or stalled')
            for arm in arms:
                if not np.all(np.isfinite(arm.get_joint_positions())):
                    raise RuntimeError('Non-finite joint state')

    def contacts(headers, data):
        """Record only contacts between rig namespaces, including raw solver impulses."""
        for header in headers:
            paths = [str(PhysicsSchemaTools.intToSdfPath(header.collider0)),
                     str(PhysicsSchemaTools.intToSdfPath(header.collider1))]
            belongs_first = [any(p.startswith(root + '/') for root in RIG_ROOTS) for p in paths]
            belongs_second = [p.startswith(SECOND_ROOT + '/') for p in paths]
            if not ((belongs_first[0] and belongs_second[1]) or (belongs_first[1] and belongs_second[0])):
                continue
            pair = tuple(sorted(paths))
            if header.type == ContactEventType.CONTACT_LOST:
                active_contacts.discard(pair)
                continue
            active_contacts.add(pair)
            points = data[header.contact_data_offset:header.contact_data_offset + header.num_contact_data]
            report['contact_events'].append({
                'time_s': timeline.get_current_time(), 'pair': pair,
                'impulse_norm_sum_ns': sum(float(np.linalg.norm(list(p.impulse))) for p in points),
                'min_separation_m': min((float(p.separation) for p in points), default=None)})

    try:
        STATE['phase'] = 'setup'
        report['checkpoints'].append(save_checkpoint(stage, '01-before-end-table-layout',
            'Stable stage before repositioning; prior layout measurements retained separately.'))
        # Fold the complete welded rigs dynamically before changing placement.
        # Restore their *coupled* posed specs after stop, which otherwise rewinds
        # the USD state. Moving only arm joint coordinates leaves the RG2 behind.
        for index, path in enumerate(['/World/UR5e', '/World/Arm2/UR5e']):
            if stage.GetPrimAtPath(path):
                preparing_arm = Articulation(path, name=f'prepare_end_layout_{index}')
                preparing_arm.initialize()
                preparing_arm.apply_action(ArticulationActions(joint_positions=home))
        await settle(3)
        prepared_path = save_checkpoint(stage, '01b-folded-before-placement',
            'Both complete welded rigs folded through drives before changing base placement.')
        report['checkpoints'].append(prepared_path)
        prepared = Sdf.Layer.FindOrOpen(prepared_path)
        timeline.stop()
        for _ in range(5):
            await app.next_update_async()
        root_layer = stage.GetRootLayer()
        for root in RIG_ROOTS:
            Sdf.CopySpec(prepared, root, root_layer, root)
        first = stage.GetPrimAtPath('/World/UR5e')
        previous = Gf.Vec3d(first.GetAttribute('xformOp:translate').Get())
        delta = Gf.Vec3d(*FIRST_BASE_TRANSLATION_M) - previous
        for root in RIG_ROOTS:
            attr = stage.GetPrimAtPath(root).GetAttribute('xformOp:translate')
            attr.Set(Gf.Vec3d(attr.Get()) + delta)
        existing = stage.GetPrimAtPath(SECOND_ROOT)
        if existing and existing.GetAttribute('rise:layout').Get() != LAYOUT:
            stage.RemovePrim(SECOND_ROOT)
        if not stage.GetPrimAtPath(SECOND_ROOT):
            parent = UsdGeom.Xform.Define(stage, SECOND_ROOT)
            # Rotate the copied first-base coordinates, then solve wrapper offset
            # to place the second base at the requested world location.
            rotated_first = Gf.Vec3d(-FIRST_BASE_TRANSLATION_M[0], -FIRST_BASE_TRANSLATION_M[1], FIRST_BASE_TRANSLATION_M[2])
            parent.AddTranslateOp().Set(Gf.Vec3d(*BASE_TRANSLATION_M) - rotated_first)
            parent.AddOrientOp().Set(Gf.Quatf(0, 0, 0, 1))
            parent.GetPrim().CreateAttribute('rise:layout', Sdf.ValueTypeNames.String).Set(LAYOUT)
            for source in RIG_ROOTS:
                destination = SECOND_ROOT + '/' + source.rsplit('/', 1)[1]
                if not root_layer.GetPrimAtPath(source):
                    raise RuntimeError(f'No authored rig spec at {source}')
                if not Sdf.CopySpec(root_layer, source, root_layer, destination):
                    raise RuntimeError(f'Could not copy {source}')
            for prim in Usd.PrimRange(stage.GetPrimAtPath(SECOND_ROOT)):
                for relationship in prim.GetRelationships():
                    targets = []
                    for target in relationship.GetTargets():
                        for source in RIG_ROOTS:
                            if target.HasPrefix(Sdf.Path(source)):
                                target = target.ReplacePrefix(Sdf.Path(source),
                                         Sdf.Path(SECOND_ROOT + '/' + source.rsplit('/', 1)[1]))
                                break
                        targets.append(target)
                    if targets:
                        relationship.SetTargets(targets)
            # PhysX authoring may recompute frames when body relationships are
            # reassigned at a posed configuration. Restore canonical joint
            # frames after all remapping, or a reload double-counts joint angle.
        report['canonical_joint_frame_count'] = restore_second_rig_joint_frames(stage)
        report['external_physics_relationships'] = []
        for prim in Usd.PrimRange(stage.GetPrimAtPath(SECOND_ROOT)):
            for relationship in prim.GetRelationships():
                if relationship.GetName() in ('physics:body0', 'physics:body1'):
                    for target in relationship.GetTargets():
                        if not target.HasPrefix(Sdf.Path(SECOND_ROOT)):
                            report['external_physics_relationships'].append(str(target))
        if report['external_physics_relationships']:
            raise RuntimeError('Second rig contains a joint targeting the first rig')
        for root in (*RIG_ROOTS, SECOND_ROOT):
            for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0)
        for root in ['/World/UR5e', '/World/Arm2/UR5e']:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
                if prim.IsA(UsdPhysics.RevoluteJoint):
                    prim.GetAttribute('state:angular:physics:velocity').Set(0.0)
        report['arm_colliders'] = {
            root: [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies())
                   if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
            for root in ['/World/UR5e', '/World/Arm2/UR5e']}
        subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contacts)
        report['overview_camera'] = create_overview_camera(stage)
        render_product = rep.create.render_product(report['overview_camera'], (960, 720),
                                                    force_new=True, name='RISE_DualArmValidation')
        rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb.attach([render_product.path])
        timeline.play()
        await settle(.5)
        for index, root in enumerate(['/World/UR5e', '/World/Arm2/UR5e']):
            arm = Articulation(root, name=f'dual_validation_{index}')
            arm.initialize()
            arms.append(arm)
            arm.apply_action(ArticulationActions(joint_positions=home))
        await settle(3)
        report['initial_home_errors_rad'] = [float(np.max(np.abs(a.get_joint_positions() - home))) for a in arms]
        report['initial_inter_rig_contacts'] = sorted(active_contacts)
        if active_contacts:
            raise RuntimeError('Rig collision at the separated home poses')
        report['checkpoints'].append(save_checkpoint(stage, '02-end-table-setup',
            'Opposite-end placement at separated folded home; movement/contact suite pending.'))
        STATE['phase'] = 'independent motion'
        for index, arm in enumerate(arms):
            for joint_index, joint_name in enumerate(arm.dof_names):
                for delta in [-.03, .03]:
                    target = home.copy()
                    target[0, joint_index] += delta
                    arm.apply_action(ArticulationActions(joint_positions=target))
                    await settle(2)
                    errors = [float(np.max(np.abs(a.get_joint_positions() - (target if i == index else home))))
                              for i, a in enumerate(arms)]
                    report['movement'].append({'arm': index + 1, 'joint': joint_name,
                                                'delta_rad': delta, 'errors_both_arms_rad': errors,
                                                'pass': max(errors) < .001 and not active_contacts})
                    arm.apply_action(ArticulationActions(joint_positions=home))
                    await settle(.25)
                    STATE['completed_movement_cases'] = len(report['movement'])
        Image.fromarray(rgb.get_data()).save(str(output / 'home.png'))
        STATE['phase'] = 'controlled inter-arm contact'
        report['contact_events'].clear()
        for angle in np.linspace(HOME_RAD[1], -.20, 121):
            target = home.copy()
            target[0, 1] = angle
            for arm in arms:
                arm.apply_action(ArticulationActions(joint_positions=target))
            await settle(.05)
            if report['contact_events']:
                await settle(.15)
                report['contact_target_rad'] = target.tolist()
                report['contact_actual_rad'] = [arm.get_joint_positions().tolist() for arm in arms]
                report['contact_tracking_errors_rad'] = [float(np.max(np.abs(a.get_joint_positions() - target))) for a in arms]
                break
        Image.fromarray(rgb.get_data()).save(str(output / 'contact.png'))
        report['collision_pass'] = bool(report['contact_events']) and any(
            event['impulse_norm_sum_ns'] > 1e-7 for event in report['contact_events'])
        STATE['phase'] = 'withdrawal'
        for arm in arms:
            arm.apply_action(ArticulationActions(joint_positions=home))
        await settle(3)
        report['return_errors_rad'] = [float(np.max(np.abs(a.get_joint_positions() - home))) for a in arms]
        report['contacts_after_withdrawal'] = sorted(active_contacts)
        report['status'] = 'complete'
        report['pass'] = (len(report['movement']) == 24 and all(x['pass'] for x in report['movement'])
                          and report['collision_pass'] and not active_contacts
                          and max(report['return_errors_rad']) < .001)
        # Export authored opinions/references, not a costly flattened CAD bundle.
        save_scene_overlay(stage, output / 'dual_arm_scene.usda')
        if report['pass']:
            report['checkpoints'].append(save_checkpoint(stage, '03-end-table-validated',
                'All 24 independent joint cases and inter-arm contact/withdrawal passed; arms at separated home.'))
    except Exception:
        report['status'] = 'error'
        report['error'] = traceback.format_exc()
        report['pass'] = False
        for arm in arms:
            if np.all(np.isfinite(arm.get_joint_positions())):
                arm.apply_action(ArticulationActions(joint_positions=home))
        if timeline.is_playing():
            await settle(2)
    finally:
        subscription = None
        if rgb is not None:
            rgb.detach([render_product.path])
        if render_product is not None:
            render_product.destroy()
        STATE['phase'] = 'finished'
        (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report
