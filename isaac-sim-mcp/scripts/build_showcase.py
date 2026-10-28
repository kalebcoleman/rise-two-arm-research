"""Build a portable presentation start scene with USD Python (no GPU needed).

This changes the presentation environment only. It does not claim a new
physics validation. Run launch_showcase.py to test and record that scene.
"""
import argparse
import json
import shutil
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdUtils


def build(source, output):
    output.mkdir(parents=True, exist_ok=True)
    # Import into a scratch layer so the accepted checkpoint is never edited.
    layer = Sdf.Layer.CreateAnonymous('presentation.usda')
    layer.ImportFromString(source.read_text())
    assets = Path(__file__).resolve().parents[1] / 'exts' / 'ur5e-dt' / 'assets'
    def resolve(path):
        if '/exts/ur5e-dt/assets/' in path:
            return str(assets / path.split('/exts/ur5e-dt/assets/', 1)[1])
        return path.removeprefix('file://')
    UsdUtils.ModifyAssetPaths(layer, resolve)
    stage = Usd.Stage.Open(layer, Usd.Stage.LoadNone)
    for path in ['/World/scene', '/Graph', '/Environment', '/World/Cube',
                 '/World/LegoWorkspace']:
        stage.RemovePrim(path)
    stage.Load()
    table = UsdGeom.Cube.Define(stage, '/World/PresentationTable')
    table.CreateSizeAttr(1)
    table.AddTranslateOp().Set(Gf.Vec3d(0, -.3, -.04))
    table.AddScaleOp().Set(Gf.Vec3f(2, 1, .08))
    table.CreateDisplayColorAttr([Gf.Vec3f(.7, .72, .74)])
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    light = UsdLux.DomeLight.Define(stage, '/World/PresentationLight')
    light.CreateIntensityAttr(1100)
    key = UsdLux.DistantLight.Define(stage, '/World/PresentationKey')
    key.CreateIntensityAttr(2500)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-35, -25, 0))
    errors = [str(e) for p in stage.Traverse() for e in p.GetPrimIndex().localErrors]
    if errors:
        raise RuntimeError(errors)
    # Flatten embeds robot/gripper/bench geometry and removes machine paths.
    flattened = stage.Flatten()
    shutil.copytree(assets / 'workstation' / 'NVIDIA', output / 'materials' / 'NVIDIA', dirs_exist_ok=True)
    UsdUtils.ModifyAssetPaths(flattened, lambda p: './materials/NVIDIA/' + p.split('/NVIDIA/', 1)[1]
                             if '/NVIDIA/' in p else p)
    flattened.customLayerData = {'risePresentation': 'Portable physics start; runtime validation pending'}
    text = flattened.ExportToString()
    if '/home/' in text or 'file://' in text:
        raise RuntimeError('Machine-specific paths remain: ' + '\n'.join(
            line for line in text.splitlines() if '/home/' in line or 'file://' in line))
    flattened.Export(str(output / 'scene.usdc'))
    start = Usd.Stage.CreateNew(str(output / 'Start Physics.usda'))
    start.GetRootLayer().subLayerPaths = ['./scene.usdc']
    start.SetStartTimeCode(0)
    start.SetEndTimeCode(3600)
    start.SetTimeCodesPerSecond(60)
    start.GetRootLayer().Save()
    (output / 'build.json').write_text(json.dumps({
        'source_checkpoint': source.name, 'composition_errors': errors,
        'physics_runtime_verified': False,
        'environment_changes': 'Removed room, ROS graphs and loose parts; added tabletop with top at z=0 and studio lights.',
        'motion': 'Requires launch_showcase.py; this file alone is a held-pose start.'
    }, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    build(args.source.resolve(), args.output.resolve())
