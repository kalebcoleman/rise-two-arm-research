"""Adaptive mass/speed tests of the physical two-gripper shared-bar grasp.

Run in Isaac Sim through MCP. Geometry/friction/controller caps stay fixed;
only object mass/inertia and requested trajectory durations change. Each trial
starts from the immutable accepted grasp stage. No physical robot driver.
"""

import json
from pathlib import Path
import shutil
import types

import numpy as np
import omni.kit.app
import omni.physx
from omni.physx.bindings._physx import ContactEventType
import omni.timeline
import omni.usd
from pxr import Gf, PhysicsSchemaTools, Sdf, Usd, UsdPhysics
from isaacsim.core.prims import RigidPrim
from scipy.spatial.transform import Rotation

CHECKPOINT_DIRECTORY = Path(__file__).resolve().parents[2] / 'checkpoints'
BASELINE = CHECKPOINT_DIRECTORY / '20261027T162819.114595Z-06-shared-grasp-lift-validated.usda'
APPROVED = CHECKPOINT_DIRECTORY / 'Approved'
STATE = {'phase': 'idle', 'trials': []}
BASE_GOALS = [
    ('translate', [0, .05, .03], 0, 0, 4),
    ('yaw', [0, .05, .03], 10, 0, 4),
    ('pitch', [0, .05, .03], 10, 5, 4),
    ('return', [0, 0, 0], 0, 0, 5),
]


async def run(output_directory, shared, checkpoints):
    """Find transition brackets with coarse sweeps, two refinements and boundary repeats.

    Passing means <=2 mm / 1 degree relative grasp drift, <=10 mm / 5 degree
    object tracking error, >=50 mm clearance, >=95% sampled four-finger contact,
    no unexpected collisions, and successful saved-stage replay. These are
    operational simulated limits, not physical payload ratings.
    """
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    results = []
    report = {'status': 'running', 'baseline': str(BASELINE), 'trials': results,
              'fixed_geometry_m': [.65, .035, .06], 'slip_onset_m': .002,
              'rotation_slip_onset_deg': 1, 'boundaries': {}}
    script = Path(__file__).with_name('coordinated_bar_motion.py')

    async def trial(mass, speed, group, repeat=False):
        """Run one isolated condition and retain failures as well as verified successes."""
        tag = f'{len(results)+1:02d}_{mass:g}kg_{speed:g}x'
        folder = output / tag
        folder.mkdir()
        STATE.update({'phase': group, 'condition': tag})
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        # A unique layer prevents dirty cached stages from contaminating a reset.
        layer = Sdf.Layer.CreateAnonymous('load_test.usda')
        layer.ImportFromString(BASELINE.read_text())
        setup = Usd.Stage.Open(layer, load=Usd.Stage.LoadAll)
        obj = setup.GetPrimAtPath(shared.OBJECT)
        material = UsdPhysics.MassAPI(obj)
        material.GetMassAttr().Set(float(mass))
        inertia = mass / 12 * np.array([.035**2 + .06**2, .65**2 + .06**2, .65**2 + .035**2])
        material.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*inertia))
        origin = shared.matrix(setup, shared.OBJECT)
        origin[:3, :3] /= np.linalg.norm(origin[:3, :3], axis=0)
        references = [np.linalg.inv(shared.grasp_frame(setup, gp)) @ origin for gp in shared.GRIPS]
        layer.Export(str(folder / 'setup.usda'))
        await omni.usd.get_context().open_stage_async(str(folder / 'setup.usda'))
        timeline.play()
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()
        motion = types.ModuleType('load_motion_' + str(len(results)))
        motion.__file__ = str(script)
        exec(compile(script.read_text(), str(script), 'exec'), motion.__dict__)
        motion.GOALS = [(name, delta, yaw, pitch, duration / speed)
                        for name, delta, yaw, pitch, duration in BASE_GOALS]
        STATE['motion'] = motion.STATE
        raw = await motion.run(str(folder), shared, checkpoints, command_dt=1/60,
                               slip_limit_m=.002, rotation_slip_limit_deg=1,
                               origin_override=origin, references_override=references,
                               checkpoint_tag=f'load-{mass:g}kg-speed-{speed:g}x')
        samples = raw['samples'] or ([raw['initial']] if 'initial' in raw else [])
        entry = {'id': tag, 'mass_kg': mass, 'requested_speed_scale': speed,
                 'group': group, 'repeat': repeat, 'pass': bool(raw['pass']),
                 'report': str(folder / 'report.json'), 'error': raw.get('error')}
        if samples:
            entry.update({'max_slip_m': max(max(x['translation_slip_m']) for x in samples),
                          'max_rotation_slip_deg': max(max(x['rotation_slip_deg']) for x in samples),
                          'max_tracking_error_m': max(x['pose_error_m'] for x in samples),
                          'max_tracking_error_deg': max(x['pose_error_deg'] for x in samples),
                          'minimum_clearance_m': min(x['bottom_z_m'] for x in samples),
                          'contact_fraction': float(np.mean([all(all(p) for p in x['finger_contacts']) for x in samples]))})
            positions = np.array([x['object_position_m'] for x in samples])
            times = np.array([x['time_s'] for x in samples])
            if len(samples) > 1:
                dt = np.diff(times)
                good = dt > 1e-8
                speeds = np.linalg.norm(np.diff(positions, axis=0)[good] / dt[good, None], axis=1)
                entry['peak_measured_linear_speed_m_s'] = float(np.max(speeds)) if len(speeds) else 0
                rotations = [np.array(x['object_rotation_matrix']) for x in samples]
                rates = [np.degrees(np.linalg.norm(Rotation.from_matrix(b @ a.T).as_rotvec())) / d
                         for a, b, d in zip(rotations[:-1], rotations[1:], dt) if d > 1e-8]
                entry['peak_measured_angular_speed_deg_s'] = float(max(rates, default=0))
                entry['measured_duration_s'] = float(times[-1] - times[0])
        if not raw['pass']:
            if entry.get('max_slip_m', 0) > .01 or entry.get('minimum_clearance_m', 1) < .05:
                reason = 'gross slip / support clearance loss'
            elif entry.get('max_slip_m', 0) > .002 or entry.get('max_rotation_slip_deg', 0) > 1:
                reason = 'relative grasp drift onset'
            elif entry.get('max_tracking_error_m', 0) > .01 or entry.get('max_tracking_error_deg', 0) > 5:
                reason = 'arm/object tracking limit; distinct from grip slip'
            elif raw['unexpected_contacts']:
                reason = 'unexpected contact / dropped bar'
            else:
                reason = 'contact loss, replay, or simulation failure'
            entry['failure_class'] = reason
        else:
            # Replay the exact final saved stage, not only its pending milestone.
            path = raw['validated_checkpoint']
            timeline.stop()
            await omni.usd.get_context().open_stage_async(path)
            timeline.play()
            for _ in range(15):
                await omni.kit.app.get_app().next_update_async()
            body = RigidPrim(shared.OBJECT, name='load_final_' + str(len(results)))
            body.initialize()
            start = timeline.get_current_time()
            heights = []
            while timeline.get_current_time() - start < 1 - 1e-9:
                await omni.kit.app.get_app().next_update_async()
                position = body.get_world_poses()[0][0]
                if not np.all(np.isfinite(position)):
                    break
                heights.append(float(position[2]))
            entry['exact_checkpoint_replay_min_center_z_m'] = min(heights, default=-1)
            if not heights or min(heights) < .08:
                entry['pass'] = False
                entry['failure_class'] = 'exact final checkpoint lost its hold'
            else:
                category = 'Heavier Bars' if group.startswith('mass') else ('Faster Moves' if group.startswith('speed') else 'Combined Loads')
                target = APPROVED / category
                target.mkdir(parents=True, exist_ok=True)
                label = f'{mass:g}kg - {speed:g}x'
                friendly = target / (label + '.usda')
                shutil.copyfile(path, friendly)
                entry['checkpoint'] = path
                entry['friendly_checkpoint'] = str(friendly)
                notes = target / 'Test Notes'
                notes.mkdir(exist_ok=True)
                (notes / (label + '.json')).write_text(json.dumps(entry, indent=2))
        results.append(entry)
        STATE['trials'] = [{k: x.get(k) for k in ['id', 'mass_kg', 'requested_speed_scale', 'pass', 'failure_class']} for x in results]
        (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        return entry

    async def boundary(axis, values, fixed):
        """Bracket the first failure and refine twice, then repeat both endpoints."""
        low, high = None, None
        for value in values:
            result = await trial(value if axis == 'mass' else fixed,
                                 fixed if axis == 'mass' else value, axis + ' coarse')
            if result['pass']:
                low = value
            else:
                high = value
                break
        if low is not None and high is not None:
            for _ in range(2):
                midpoint = (low + high) / 2
                result = await trial(midpoint if axis == 'mass' else fixed,
                                     fixed if axis == 'mass' else midpoint, axis + ' refine')
                if result['pass']:
                    low = midpoint
                else:
                    high = midpoint
            repeated = []
            for value in [low, high]:
                result = await trial(value if axis == 'mass' else fixed,
                                     fixed if axis == 'mass' else value, axis + ' repeat', repeat=True)
                repeated.append({'value': value, 'pass': result['pass'], 'failure_class': result.get('failure_class')})
            report['boundaries'][axis] = {'last_pass': low, 'first_fail': high, 'fixed_other_parameter': fixed,
                                          'endpoint_repeats': repeated}
        else:
            report['boundaries'][axis] = {'last_pass': low, 'first_fail': high, 'fixed_other_parameter': fixed}
        (output / 'summary.json').write_text(json.dumps(report, indent=2))
        return low, high

    try:
        mass_low, mass_high = await boundary('mass', [.1, .25, .5, 1, 2, 4, 8, 16, 32], 1)
        speed_low, speed_high = await boundary('speed', [1, 2, 4, 8, 16, 32, 64], .1)
        if mass_low is not None and speed_low is not None:
            for mass, speed in [(mass_low, 2), (mass_low / 2, speed_low), (mass_low, speed_low)]:
                if not any(abs(x['mass_kg']-mass)<1e-9 and abs(x['requested_speed_scale']-speed)<1e-9 for x in results):
                    await trial(mass, speed, 'combined')
        report['status'] = 'complete'
        await verify_approved(report, output, shared)
    except Exception:
        import traceback
        report['status'] = 'error'
        report['error'] = traceback.format_exc()
    finally:
        # Leave a proven condition rather than a failed stress pose on screen.
        successful = [r for r in results if r['pass'] and 'checkpoint' in r]
        if successful:
            chosen = successful[-1]
            timeline = omni.timeline.get_timeline_interface()
            timeline.stop()
            await omni.usd.get_context().open_stage_async(chosen['checkpoint'])
            timeline.play()
            report['left_on_checkpoint'] = chosen['checkpoint']
        (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        STATE['phase'] = 'finished'
    return report


async def verify_approved(report, output_directory, shared):
    """Replay every distinct browser checkpoint and verify contact, clearance and slip.

    This supplements the motion runner's pending-stage replay and the coarse
    center-height reset check. The exact human-readable file must retain both
    finger pairs, without support contacts, while gravity stays enabled.
    """
    output = Path(output_directory)
    chosen = {r['friendly_checkpoint']: r for r in report['trials']
              if r['pass'] and 'friendly_checkpoint' in r}
    verification = []
    for path, entry in chosen.items():
        STATE.update({'phase': 'verify approved checkpoints', 'condition': Path(path).name})
        t = omni.timeline.get_timeline_interface()
        t.stop()
        await omni.usd.get_context().open_stage_async(path)
        stage = omni.usd.get_context().get_stage()
        origin = shared.matrix(stage, shared.OBJECT)
        origin[:3, :3] /= np.linalg.norm(origin[:3, :3], axis=0)
        refs = [np.linalg.inv(shared.grasp_frame(stage, gp)) @ origin for gp in shared.GRIPS]
        active = set()
        support_contacts = set()

        def callback(headers, data):
            """Collect object contacts for this exact saved-stage playback."""
            for h in headers:
                pair = tuple(sorted([str(PhysicsSchemaTools.intToSdfPath(h.collider0)),
                                     str(PhysicsSchemaTools.intToSdfPath(h.collider1))]))
                if shared.OBJECT not in pair:
                    continue
                if h.type == ContactEventType.CONTACT_LOST:
                    active.discard(pair)
                else:
                    active.add(pair)
                    other = pair[1] if pair[0] == shared.OBJECT else pair[0]
                    if not any('/' + finger + '/' in other for finger in ['rg2_l_finger_link', 'rg2_r_finger_link']):
                        support_contacts.add(pair)

        sub = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(callback)
        values = []
        try:
            t.play()
            for _ in range(15):
                await omni.kit.app.get_app().next_update_async()
            body = RigidPrim(shared.OBJECT, name='approved_verification_' + str(len(verification)))
            body.initialize()
            start = t.get_current_time()
            while t.get_current_time() - start < 1 - 1e-9:
                await omni.kit.app.get_app().next_update_async()
                p, q = body.get_world_poses()
                transform = np.eye(4)
                transform[:3, :3] = Rotation.from_quat([q[0,1], q[0,2], q[0,3], q[0,0]]).as_matrix()
                transform[:3, 3] = p[0]
                slip = [float(np.linalg.norm((np.linalg.inv(shared.grasp_frame(stage, gp)) @ transform)[:3,3] - ref[:3,3]))
                        for gp, ref in zip(shared.GRIPS, refs)]
                flags = [[any(gp + '/' + finger + '/' in str(pair) for pair in active)
                          for finger in ['rg2_l_finger_link', 'rg2_r_finger_link']] for gp in shared.GRIPS]
                values.append({'clearance_m': float(p[0,2] - np.abs(transform[2,:3]) @ np.array([.325,.0175,.03])),
                               'slip_m': max(slip), 'four_finger_contact': all(all(x) for x in flags)})
            obj = stage.GetPrimAtPath(shared.OBJECT)
            result = {'file': path, 'mass_kg': entry['mass_kg'], 'speed_scale': entry['requested_speed_scale'],
                      'actual_mass_kg': float(UsdPhysics.MassAPI(obj).GetMassAttr().Get()),
                      'minimum_clearance_m': min(v['clearance_m'] for v in values),
                      'maximum_slip_m': max(v['slip_m'] for v in values),
                      'four_finger_contact_fraction': float(np.mean([v['four_finger_contact'] for v in values])),
                      'support_contacts': sorted(support_contacts),
                      'gravity_disabled': obj.GetAttribute('physxRigidBody:disableGravity').Get(),
                      'kinematic': UsdPhysics.RigidBodyAPI(obj).GetKinematicEnabledAttr().Get()}
            result['pass'] = (result['minimum_clearance_m'] >= .05 and result['maximum_slip_m'] <= .002
                              and result['four_finger_contact_fraction'] >= .95 and not support_contacts
                              and not result['gravity_disabled'] and not result['kinematic']
                              and abs(result['actual_mass_kg'] - entry['mass_kg']) < 1e-6)
        except Exception:
            import traceback
            result = {'file': path, 'pass': False, 'error': traceback.format_exc()}
        finally:
            sub = None
        verification.append(result)
        entry['approved_file_replay'] = result
        if not result['pass']:
            entry['pass'] = False
            entry['failure_class'] = 'approved browser file failed strict replay'
            rejected = output / 'rejected-browser-copies'
            rejected.mkdir(exist_ok=True)
            shutil.move(path, rejected / Path(path).name)
        notes = Path(path).parent / 'Test Notes'
        notes.mkdir(exist_ok=True)
        (notes / (Path(path).stem + '.json')).write_text(json.dumps(entry, indent=2))
    report['approved_checkpoint_replays'] = verification
    (output / 'approved-checkpoint-replays.json').write_text(json.dumps(verification, indent=2))
    (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return verification
