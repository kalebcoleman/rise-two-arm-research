"""Rebuild pinned upstream checkouts and overlay the preserved local research."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--destination', type=Path, required=True)
parser.add_argument('--source-cache', type=Path, help='Optional existing workspace for local clone verification')
args = parser.parse_args()
destination = args.destination.expanduser().resolve()
if destination.exists():
    raise SystemExit('Choose a new destination; existing workspaces are never overwritten.')
destination.mkdir(parents=True)
lock = json.loads((ROOT / 'upstream-lock.json').read_text())
for name, spec in lock.items():
    checkout = destination / name
    source = str(args.source_cache / name) if args.source_cache else spec['url']
    subprocess.run(['git', 'clone', '--no-checkout', source, str(checkout)], check=True)
    subprocess.run(['git', '-C', str(checkout), 'checkout', '--detach', spec['commit']], check=True)
    if args.source_cache:
        subprocess.run(['git', '-C', str(checkout), 'remote', 'set-url', 'origin', spec['url']], check=True)
    patch = ROOT / 'patches' / (name + '.patch')
    if patch.exists() and patch.stat().st_size:
        subprocess.run(['git', '-C', str(checkout), 'apply', '--check', str(patch)], check=True)
        subprocess.run(['git', '-C', str(checkout), 'apply', str(patch)], check=True)
    overlay = ROOT / name
    if overlay.exists():
        shutil.copytree(overlay, checkout, dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('__pycache__', '*.log'))
for name in ['checkpoints', 'START_HERE.md', 'POST_CLONE_CHANGES.md']:
    source = ROOT / name
    if source.is_dir():
        shutil.copytree(source, destination / name)
    else:
        shutil.copy2(source, destination / name)
derived = []
for stage in (destination / 'checkpoints').rglob('*.usda'):
    original = stage.read_bytes()
    text = re.sub(r'@([^@]*?)/home/kaleb/projects/RISE/([^@]*)@',
                  lambda m: '@' + m[1] + str(destination) + '/' + m[2] + '@', original.decode())
    stage.write_text(text)
    derived.append({'file': str(stage.relative_to(destination)),
                    'original_sha256': hashlib.sha256(original).hexdigest(),
                    'rebased_sha256': hashlib.sha256(stage.read_bytes()).hexdigest()})
(destination / 'checkpoint-relocation.json').write_text(json.dumps(derived, indent=2))
print(f'Workspace ready: {destination}\nInstall dependencies, then use the showcase or research scripts.')
