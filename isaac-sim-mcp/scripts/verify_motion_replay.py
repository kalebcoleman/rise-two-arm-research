"""Reopen the recorded USDA, test Play, and render start/middle/end evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--stage', type=Path)
parser.add_argument('--show', action='store_true', help='Leave a visible viewer open at the start after verification')
args = parser.parse_args()
path = (args.stage or Path(__file__).resolve().parents[1] / 'showcase/recordings/WATCH - Two Arm Motion.usda').resolve()

from isaacsim import SimulationApp
app = SimulationApp({'headless': not args.show, 'width': 1280, 'height': 720})
import numpy as np
import omni.usd
import omni.timeline
import omni.replicator.core as rep
import omni.kit.viewport.utility as viewport
from PIL import Image
from pxr import Usd, UsdGeom, UsdPhysics

omni.usd.get_context().open_stage(str(path))
stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()
active = viewport.get_active_viewport()
if active:
    active.camera_path = '/World/dual_arm_overview'
errors = [str(e) for p in stage.Traverse() for e in p.GetPrimIndex().localErrors]
assert not errors, errors
assert not any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse())
bar = stage.GetPrimAtPath('/World/SharedBar')
duration = stage.GetEndTimeCode() / stage.GetTimeCodesPerSecond()
first = np.array(UsdGeom.XformCache(Usd.TimeCode(0)).GetLocalToWorldTransform(bar))[3, :3]
timeline.set_current_time(0)
timeline.play()
for _ in range(1200):
    app.update()
    if timeline.get_current_time() >= 2:
        break
elapsed = timeline.get_current_time()
later = np.array(UsdGeom.XformCache(Usd.TimeCode(elapsed * 60)).GetLocalToWorldTransform(bar))[3, :3]
assert elapsed >= 2 and np.linalg.norm(later - first) > 1e-5
timeline.pause()
product = rep.create.render_product('/World/dual_arm_overview', (960, 720))
rgb = rep.AnnotatorRegistry.get_annotator('rgb')
rgb.attach([product.path])
poses = []
for label, seconds in [('start', 0), ('middle', duration / 2), ('end', duration)]:
    timeline.set_current_time(seconds)
    for _ in range(4):
        app.update()
    rep.orchestrator.step(rt_subframes=4, delta_time=0)
    data = rgb.get_data()
    assert data.ndim == 3 and data.shape[0] == 720
    Image.fromarray(data).save(str(path.parent / ('replay-' + label + '.png')))
    position = np.array(UsdGeom.XformCache(Usd.TimeCode(seconds * 60)).GetLocalToWorldTransform(bar))[3, :3]
    poses.append({'label': label, 'time_s': seconds, 'bar_position_m': position.tolist()})
rgb.detach([product.path])
product.destroy()
report = {'pass': True, 'stage': path.name, 'duration_s': duration,
          'play_advanced_to_s': elapsed, 'bar_movement_during_play_m': float(np.linalg.norm(later - first)),
          'composition_errors': errors, 'physics_disabled': True, 'rendered_poses': poses,
          'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in [path, path.parent / 'Recorded Motion.usdc']}}
(path.parent / 'playback-verification.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2), flush=True)
timeline.set_current_time(0)
if args.show:
    print('Viewer ready. Press Play to watch the recorded experiment.', flush=True)
    while app.is_running():
        app.update()
app.close()
