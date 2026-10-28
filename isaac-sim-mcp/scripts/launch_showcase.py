"""Isaac Sim 5.1 entry point: Play runs physics; passing runs export a USD replay.

Run using Isaac Sim's python.sh or the project's Isaac-enabled Python. A replay
is recorded link/object motion with physics disabled, not a new physics test.
"""
import argparse
import asyncio
import json
import shutil
from pathlib import Path
import sys
import types

parser = argparse.ArgumentParser()
parser.add_argument('--headless', action='store_true')
parser.add_argument('--record', action='store_true', help='Run once automatically, write replay, then exit')
parser.add_argument('--output', type=Path)
args = parser.parse_args()
ROOT = Path(__file__).resolve().parents[1]
OUTPUT = (args.output or ROOT / 'showcase' / 'recordings').resolve()
OUTPUT.mkdir(parents=True, exist_ok=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({'headless': args.headless, 'width': 1280, 'height': 720})

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils
from isaacsim.core.api import World


def load_script(name, filename):
    module = types.ModuleType(name)
    module.__file__ = str(ROOT / 'scripts' / filename)
    sys.modules[name] = module
    exec(compile(Path(module.__file__).read_text(), module.__file__, 'exec'), module.__dict__)
    return module


class Recording:
    def __init__(self):
        self.start = None
        self.last = -1
        self.frames = []
        self.attributes = None

    def sample(self, stage, seconds):
        if self.start is None:
            self.start = seconds
            self.attributes = [a for p in stage.Traverse()
                               if p.HasAPI(UsdPhysics.RigidBodyAPI)
                               for a in p.GetAttributes()
                               if a.GetName() in ('xformOp:translate', 'xformOp:orient')]
        frame = round((seconds - self.start) * 60)
        if frame <= self.last:
            return
        values = [(str(a.GetPath()), a.Get()) for a in self.attributes]
        for path, value in values:
            data = ([value.GetReal(), *value.GetImaginary()]
                    if hasattr(value, 'GetReal') else list(value))
            if not np.all(np.isfinite(data)):
                raise RuntimeError(f'Non-finite recorded transform: {path}')
        self.frames.append((frame, values))
        self.last = frame

    def export(self, stage):
        if len(self.frames) < 2:
            raise RuntimeError('No motion recorded')
        layer = stage.Flatten()
        shutil.copytree(ROOT / 'showcase' / 'materials', OUTPUT / 'materials', dirs_exist_ok=True)
        UsdUtils.ModifyAssetPaths(layer, lambda p: './materials/' + p.split('/materials/', 1)[1]
                                 if '/materials/' in p else p)
        baked = Usd.Stage.Open(layer)
        # Runtime render products and their OmniGraphs must be recreated by
        # the viewing app; serialized graphs keep stale renderer handles.
        for path in ['/Render', '/Replicator', '/Orchestrator']:
            baked.RemovePrim(path)
        for prim in list(baked.Traverse()):
            # Remove simulation schemas from the presentation copy only.
            schemas = [s for s in prim.GetAppliedSchemas() if 'Physics' not in s and 'Physx' not in s]
            prim.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(schemas))
            if prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdPhysics.Scene):
                prim.SetActive(False)
        for frame, values in self.frames:
            for path, value in values:
                baked.GetAttributeAtPath(path).Set(value, Usd.TimeCode(frame))
        baked.SetStartTimeCode(0)
        baked.SetEndTimeCode(self.last)
        baked.SetTimeCodesPerSecond(60)
        baked.SetFramesPerSecond(60)
        layer.customLayerData = {'riseReplay': 'Recorded passing physics run; physics disabled for presentation playback'}
        # Keep the readable USDA entry small; binary USD holds geometry and
        # sampled transforms below GitHub's per-file size limit.
        layer.Export(str(OUTPUT / 'Recorded Motion.usdc'))
        destination = OUTPUT / 'WATCH - Two Arm Motion.usda'
        entry = Sdf.Layer.CreateAnonymous('watch.usda')
        entry.subLayerPaths = ['./Recorded Motion.usdc']
        entry.defaultPrim = 'World'
        entry.startTimeCode = 0
        entry.endTimeCode = self.last
        entry.timeCodesPerSecond = 60
        entry.framesPerSecond = 60
        entry.customLayerData = layer.customLayerData
        entry.Export(str(destination))
        cached = Sdf.Layer.Find(str(destination))
        if cached:
            cached.Reload()
        check = Usd.Stage.Open(str(destination))
        if any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in check.Traverse()):
            raise RuntimeError('Replay still has an enabled rigid-body schema')
        bar = check.GetAttributeAtPath(shared.OBJECT + '.xformOp:translate')
        if not bar or len(bar.GetTimeSamples()) < 2:
            raise RuntimeError('Replay has no sampled bar translation')
        return destination


timeline = omni.timeline.get_timeline_interface()
state = {'busy': False, 'complete': False, 'ready': False, 'message': 'Loading', 'exit_code': 1}
motion = load_script('rise_showcase_motion', 'coordinated_bar_motion.py')
shared = load_script('rise_showcase_shared', 'shared_object_experiment.py')


def reset():
    if state['busy']:
        return
    state.update(ready=False, complete=False, message='Loading start pose')
    timeline.stop()
    omni.usd.get_context().open_stage(str(ROOT / 'showcase' / 'Start Physics.usda'))
    simulation_app.update()
    world = World.instance() or World(physics_dt=1 / 120, rendering_dt=1 / 60)
    import omni.kit.viewport.utility as viewport
    active = viewport.get_active_viewport()
    if active:
        active.camera_path = '/World/dual_arm_overview'
    state.update(ready=True, message='Ready: press Play to run physics')
    print(state['message'], flush=True)


async def run():
    state.update(busy=True, message='Running physics and recording measured transforms')
    recording = Recording()
    try:
        # Let PhysX create tensor views before initializing the articulations.
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()
        result = await motion.run(OUTPUT, shared, None, command_dt=1 / 60,
                                  slip_limit_m=.002, rotation_slip_limit_deg=1,
                                  save_checkpoints=False, sample_callback=recording.sample)
        if not result['pass']:
            raise RuntimeError(result.get('error', 'Physics validation failed'))
        replay = recording.export(omni.usd.get_context().get_stage())
        # Actually reopen and advance the exported file's timeline before
        # calling it Play-ready. The measured physics report remains separate.
        timeline.stop()
        await omni.usd.get_context().open_stage_async(str(replay))
        timeline.set_current_time(0)
        replay_stage = omni.usd.get_context().get_stage()
        first = np.array(UsdGeom.XformCache(Usd.TimeCode(0)).GetLocalToWorldTransform(
            replay_stage.GetPrimAtPath(shared.OBJECT)))[3, :3]
        timeline.play()
        updates = 0
        while timeline.get_current_time() < min(2, recording.last / 120) and updates < 1200:
            await omni.kit.app.get_app().next_update_async()
            updates += 1
        later = np.array(UsdGeom.XformCache(Usd.TimeCode(timeline.get_current_time() * 60)).GetLocalToWorldTransform(
            replay_stage.GetPrimAtPath(shared.OBJECT)))[3, :3]
        if timeline.get_current_time() <= 0 or np.linalg.norm(later - first) < 1e-6:
            raise RuntimeError('Exported replay did not advance and move the bar')
        timeline.pause()
        timeline.set_current_time(0)
        state.update(complete=True, exit_code=0, message=f'Passed. Play-ready replay: {replay}')
        (OUTPUT / 'replay-manifest.json').write_text(json.dumps({
            'replay': replay.name, 'frames_recorded': len(recording.frames),
            'duration_s': recording.last / 60, 'physics_summary': result['summary'],
            'kind': 'Recorded measured simulation transforms, physics disabled in replay'
        }, indent=2))
        (OUTPUT / 'demo-error.txt').unlink(missing_ok=True)
    except Exception as exc:
        state.update(complete=True, exit_code=1, message=f'Demo failed; no new replay approved: {exc}')
        (OUTPUT / 'demo-error.txt').write_text(state['message'])
    finally:
        timeline.pause()
        state['busy'] = False
        print(state['message'], flush=True)


def on_timeline(event):
    if (event.type == int(omni.timeline.TimelineEventType.PLAY)
            and state['ready'] and not state['busy'] and not state['complete']):
        # Mark busy before scheduling to prevent duplicate Play events.
        state['busy'] = True
        asyncio.ensure_future(run())


subscription = timeline.get_timeline_event_stream().create_subscription_to_pop(on_timeline)
if not args.headless:
    import omni.ui as ui
    window = ui.Window('RISE Two Arm Demo', width=430, height=140)
    with window.frame:
        with ui.VStack():
            ui.Label('Press Play to move both arms with live physics.')
            ui.Label('Reset before another run. Successful runs create WATCH USDA.')
            ui.Button('Reset Demo', clicked_fn=lambda: state.update(reset_requested=True))

reset()
autostarted = False
while simulation_app.is_running():
    if state.pop('reset_requested', False):
        reset()
    simulation_app.update()
    if args.record and state['ready'] and not autostarted:
        autostarted = True
        timeline.play()
    if args.record and state['complete'] and not state['busy']:
        break
simulation_app.close()
sys.exit(state['exit_code'] if args.record else 0)
