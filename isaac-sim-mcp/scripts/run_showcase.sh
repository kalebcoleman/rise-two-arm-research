#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "$0")/.." && pwd)
runtime=${RISE_PYTHON:-"$repo/../BrickSim/.venv/bin/python"}
if [[ ! -x "$runtime" ]]; then
  echo 'Set RISE_PYTHON to Isaac Sim 5.1 python.sh (or an Isaac-enabled Python).'
  exit 1
fi
exec "$runtime" "$repo/scripts/launch_showcase.py" "$@"
