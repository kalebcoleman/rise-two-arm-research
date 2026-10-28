"""Refresh the publication snapshot from the adjacent original workspace.

Explicit paths omit environments, credentials, tunnels and unrelated scratch
files. Existing upstream checkouts and experiment evidence are not modified.
"""
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
workspace = ROOT.parent
for name in ['START_HERE.md', 'POST_CLONE_CHANGES.md']:
    shutil.copy2(workspace / name, ROOT / name)
for name in ['checkpoints', 'isaac-sim-mcp/docs', 'isaac-sim-mcp/plans',
             'isaac-sim-mcp/showcase', 'isaac-sim-mcp/resources', 'BrickSim/research',
             'BrickSim/experiments']:
    shutil.copytree(workspace / name, ROOT / name, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__', '*.log', 'demo-error.txt'))
scripts = ['bar_load_speed_limits.py', 'build_showcase.py', 'coordinated_bar_motion.py',
           'dual_arm_experiment.py', 'launch_showcase.py', 'package_showcase.py',
           'render_bar_limit_report.py', 'run_showcase.sh', 'shared_object_experiment.py',
           'validate_workstation.py', 'verify_motion_replay.py']
for name in scripts:
    target = ROOT / 'isaac-sim-mcp/scripts' / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(workspace / 'isaac-sim-mcp/scripts' / name, target)
for name in ['isaac-sim-mcp/exts/ur5e-dt/ur5e_dt/physics_setup.py',
             'isaac-sim-mcp/LICENSE', 'isaac-sim-mcp/LICENSE_README.md',
             'BrickSim/scripts/run_rise_lab.sh', 'BrickSim/scripts/run_desktop.sh',
             'BrickSim/docs/UBUNTU_SETUP.md']:
    target = ROOT / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(workspace / name, target)
lock = {}
for name in ['isaac-sim-mcp', 'BrickSim', 'omni-kit-mcp']:
    checkout = workspace / name
    lock[name] = {
        'url': subprocess.check_output(['git', '-C', str(checkout), 'remote', 'get-url', 'origin'], text=True).strip(),
        'commit': subprocess.check_output(['git', '-C', str(checkout), 'rev-parse', 'HEAD'], text=True).strip()}
    patch = subprocess.check_output(['git', '-C', str(checkout), 'diff', '--binary', 'HEAD'])
    (ROOT / 'patches' / (name + '.patch')).write_bytes(patch)
(ROOT / 'upstream-lock.json').write_text(json.dumps(lock, indent=2))
print('Publication snapshot refreshed; review and commit it before pushing.')
