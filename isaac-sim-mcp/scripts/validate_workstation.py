"""Run inside Isaac Sim via MCP: await run(output_directory).

Uses simulation time, radians for joint tensors, meters for geometry and depth,
and USD wxyz orientations. Never starts a ROS or physical robot driver.
"""

import asyncio
import json
from pathlib import Path
import traceback

import numpy as np
import omni.kit.app
import omni.physx
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from PIL import Image
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics
from isaacsim.core.prims import Articulation, RigidPrim
from isaacsim.core.utils.types import ArticulationActions
from ur5e_dt.physics_setup import (
    ROBOT_START_POSITIONS_RAD, configure_robot_start, configure_workstation_collisions,
)


async def run(output_directory):
    """Validate the existing single-arm scene and write JSON plus a camera PNG.

    Temporarily disables legacy supporting surfaces to isolate bench contact.
    Cleans probes and restores those surfaces, then rebuilds physics and verifies
    return to the raised start pose. Leaves the existing simulation playing.
    """
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'motion': [], 'progress': 'setup'}
    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()
    app = omni.kit.app.get_app()
    legacy = []
    contact_subscription = None
    render_product = None
    annotators = []
    pairs = set()
    probes = ['/World/RISE_BenchProbe', '/World/RISE_ArmProbe']
    start = np.array([ROBOT_START_POSITIONS_RAD], dtype=np.float32)

    async def settle(seconds):
        """Advance the live app until the requested simulation time has elapsed."""
        initial = timeline.get_current_time()
        for _ in range(10000):
            await app.next_update_async()
            if not timeline.is_playing():
                raise RuntimeError('Simulation timeline stopped during validation')
            if timeline.get_current_time() - initial >= seconds:
                return
        raise RuntimeError('Simulation time did not advance sufficiently')

    def contact_callback(headers, data):
        """Collect actual PhysX collider pairs without estimating force from render time."""
        for header in headers:
            pairs.add((str(PhysicsSchemaTools.intToSdfPath(header.collider0)),
                       str(PhysicsSchemaTools.intToSdfPath(header.collider1))))

    try:
        timeline.stop()
        for _ in range(5):
            await app.next_update_async()
        report['bench_collider_count'] = len(configure_workstation_collisions(stage))
        configure_robot_start(stage)
        robot_colliders = [p for p in Usd.PrimRange(stage.GetPrimAtPath('/World/UR5e'),
                            Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI)]
        report['robot_colliders'] = [str(p.GetPath()) for p in robot_colliders]
        report['robot_colliders_enabled'] = all(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()
                                                 for p in robot_colliders)
        for p in stage.Traverse():
            if p.HasAPI(UsdPhysics.CollisionAPI) and (
                str(p.GetPath()).startswith('/World/defaultGroundPlane/')
                or str(p.GetPath()) == '/World/scene/roomScene/colliders/table'
            ):
                attr = UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr()
                legacy.append((attr, attr.Get()))
                attr.Set(False)
        report['temporarily_disabled_surfaces'] = [str(a.GetPath()) for a, _ in legacy]
        for path, mass in zip(probes, [.1, .01]):
            if stage.GetPrimAtPath(path):
                raise RuntimeError(f'Probe already exists: {path}')
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(.04 if mass == .1 else .02)
            cube.AddTranslateOp().Set(Gf.Vec3d(2, 2, 1))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(mass)
            PhysxSchema.PhysxContactReportAPI.Apply(cube.GetPrim()).CreateThresholdAttr(0)
        render_product = rep.create.render_product('/World/workspace_camera_sim', (640, 480),
                                                    force_new=True, name='RISE_WorkstationValidation')
        for name in ['rgb', 'distance_to_image_plane']:
            annotator = rep.AnnotatorRegistry.get_annotator(name)
            annotator.attach([render_product.path])
            annotators.append(annotator)
        contact_subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact_callback)
        timeline.play()
        await settle(.5)
        arm = Articulation('/World/UR5e', name='workstation_validation_arm')
        arm.initialize()
        arm.apply_action(ArticulationActions(joint_positions=start))
        await settle(2)
        report['progress'] = 'motion'
        for index, name in enumerate(arm.dof_names):
            for delta in [-.03, .03]:
                target = start.copy()
                target[0, index] += delta
                arm.apply_action(ArticulationActions(joint_positions=target))
                await settle(2)
                actual = arm.get_joint_positions()
                if not np.all(np.isfinite(actual)):
                    raise RuntimeError(f'Non-finite joint state during {name} test')
                error = float(np.max(np.abs(actual - target)))
                report['motion'].append({'joint': name, 'delta_rad': delta,
                                         'max_all_joint_error_rad': error, 'pass': error < .001})
                arm.apply_action(ArticulationActions(joint_positions=start))
                await settle(.5)
        report['progress'] = 'camera'
        rgb, depth = [a.get_data() for a in annotators]
        Image.fromarray(rgb).save(str(output / 'camera.png'))
        valid = np.isfinite(depth) & (depth > 0)
        report['camera'] = {'rgb_shape': list(rgb.shape), 'rgb_std': float(rgb[:, :, :3].std()),
                            'depth_shape': list(depth.shape), 'valid_depth_fraction': float(valid.mean()),
                            'pass': bool(list(rgb.shape[:2]) == [480, 640] and float(rgb.std()) > 1 and valid.mean() > .95)}
        report['progress'] = 'bench contact'
        bench_probe = RigidPrim(probes[0], name='bench_validation_probe')
        arm_probe = RigidPrim(probes[1], name='arm_validation_probe')
        bench_probe.initialize()
        arm_probe.initialize()
        bench_probe.set_world_poses(positions=np.array([[-.5, -.4, .2]], dtype=np.float32))
        bench_probe.set_velocities(np.zeros((1, 6), dtype=np.float32))
        pairs.clear()
        heights = []
        for _ in range(120):
            await settle(1 / 120)
            heights.append(float(bench_probe.get_world_poses()[0][0, 2]))
        bench_pairs = sorted(p for p in pairs if probes[0] in p)
        expected_z = -.08547463681875278 + .02
        measured_z = heights[-1]
        report['bench_contact'] = {'expected_center_z_m': expected_z, 'actual_center_z_m': measured_z,
                                  'tail_z_range_m': max(heights[-20:]) - min(heights[-20:]),
                                  'pairs': bench_pairs,
                                  'pass': abs(measured_z - expected_z) < .002 and any('/World/Vention/Collisions/' in str(p) for p in bench_pairs)}
        report['progress'] = 'robot contact'
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        bounds = bounds.ComputeWorldBound(stage.GetPrimAtPath('/World/UR5e/forearm_link')).ComputeAlignedRange()
        center = bounds.GetMidpoint()
        query = omni.physx.get_physx_scene_query_interface().raycast_closest(
            (float(center[0]), float(center[1]), float(bounds.GetMax()[2]) + .3), (0, 0, -1), 2)
        report['arm_ray_hit'] = {k: str(v) for k, v in query.items()}
        if not query['hit'] or '/World/UR5e/' not in query['collision']:
            raise RuntimeError('Could not locate robot collision surface for probe drop')
        position = np.array([[center[0], center[1], query['position'][2] + .12]], dtype=np.float32)
        arm_probe.set_world_poses(positions=position)
        arm_probe.set_velocities(np.zeros((1, 6), dtype=np.float32))
        pairs.clear()
        await settle(1)
        arm_pairs = sorted(p for p in pairs if probes[1] in p)
        report['robot_contact'] = {'pairs': arm_pairs,
                                   'pass': any('/World/UR5e/' in str(p) for p in arm_pairs)}
        report['status'] = 'complete'
    except Exception:
        report['status'] = 'error'
        report['error'] = traceback.format_exc()
    finally:
        report['progress'] = 'cleanup'
        contact_subscription = None
        timeline.stop()
        for _ in range(5):
            await app.next_update_async()
        for annotator in annotators:
            annotator.detach([render_product.path])
        if render_product:
            render_product.destroy()
        for path in probes:
            if stage.GetPrimAtPath(path):
                stage.RemovePrim(path)
        for attr, value in legacy:
            attr.Set(value)
        configure_robot_start(stage)
        timeline.play()
        await settle(.5)
        restored_arm = Articulation('/World/UR5e', name='workstation_validation_restored')
        restored_arm.initialize()
        restored_arm.apply_action(ArticulationActions(joint_positions=start))
        await settle(2)
        report['return_error_rad'] = float(np.max(np.abs(restored_arm.get_joint_positions() - start)))
        report['probes_removed'] = all(not stage.GetPrimAtPath(p) for p in probes)
        report['legacy_surfaces_restored'] = all(a.Get() == value for a, value in legacy)
        report['progress'] = 'finished'
        report['pass'] = (report['status'] == 'complete' and len(report['motion']) == 12
                          and all(x['pass'] for x in report['motion'])
                          and report['camera']['pass'] and report['bench_contact']['pass']
                          and report['robot_contact']['pass'] and report['return_error_rad'] < .001)
        (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report
