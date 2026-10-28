"""Package the portable demo and guides, without environments or credentials."""
import hashlib
import json
from pathlib import Path
import zipfile

repo = Path(__file__).resolve().parents[1]
workspace = repo.parent
destination = workspace / 'transfers' / 'RISE-school-showcase.zip'
destination.parent.mkdir(exist_ok=True)
files = [workspace / 'START_HERE.md', workspace / 'POST_CLONE_CHANGES.md']
files += [repo / 'scripts' / name for name in (
    'launch_showcase.py', 'run_showcase.sh', 'coordinated_bar_motion.py',
    'shared_object_experiment.py', 'package_showcase.py', 'verify_motion_replay.py')]
files += [workspace / 'checkpoints/Approved/00 WATCH MOTION.usda']
files += [repo / 'LICENSE', repo / 'LICENSE_README.md']
files += list((repo / 'docs').rglob('*.md'))
files += [p for p in (repo / 'showcase').rglob('*') if p.is_file()
          and p.suffix != '.log' and p.name != 'demo-error.txt']
with zipfile.ZipFile(destination, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
    for file in sorted(set(files)):
        archive.write(file, str(file.relative_to(workspace)))
verification = repo / 'showcase/recordings/playback-verification.json'
validated = verification.exists() and json.loads(verification.read_text()).get('pass', False)
manifest = {'archive': destination.name, 'sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
            'file_count': len(set(files)), 'contains_validated_motion_replay': validated,
            'status': 'Fresh physics and recorded Play/render verified' if validated else 'Validation pending'}
destination.with_suffix('.json').write_text(json.dumps(manifest, indent=2))
print(json.dumps(manifest, indent=2))
