# RISE two-arm manipulation research

Two opposing UR5e arms with RG2 grippers manipulating a shared bar in Isaac
Sim 5.1. This repository preserves the local research additions, scene assets,
experiment results and upstream reconstruction instructions.

**Project page:** https://kalebcoleman.github.io/rise-two-arm-research/

## Show the experiment

After cloning, open `checkpoints/Approved/00 WATCH MOTION.usda` in Isaac Sim
and press **Play** to watch the included 26-second recording. The companion
USD/material files are included; keep the directory layout. To run fresh physics:

With Isaac Sim 5.1 installed and `nvidia-smi` working:

```bash
git clone https://github.com/kalebcoleman/rise-two-arm-research.git
cd rise-two-arm-research
RISE_PYTHON=/path/to/isaac-sim/python.sh ./isaac-sim-mcp/scripts/run_showcase.sh
```

Press Play in the launcher. Successful physics runs generate a separate
`WATCH - Two Arm Motion.usda` recording. See
[the demo guide](isaac-sim-mcp/docs/SHOWCASE.md) for validation status and playback.
Opening a historical checkpoint by itself restores a pose, not a trajectory.

## Understand and extend the work

- [Start here](START_HERE.md): reading order and research roadmap.
- [Verified demo baseline](docs/VALIDATION.md): test results and evidence.
- [Post-clone inventory](POST_CLONE_CHANGES.md): upstream features versus additions.
- `isaac-sim-mcp/scripts/`: grasp, kinematics, motion, load/speed tests and demo.
- `isaac-sim-mcp/docs/`: methods, measured results and limitations.
- `isaac-sim-mcp/resources/`: original experimental evidence, including failures.
- `checkpoints/Approved/`: human-readable accepted held-pose checkpoints.
- `BrickSim/research/`: early experiment protocol and notebook.
- `patches/` and `upstream-lock.json`: tracked source changes and exact upstream bases.

The two upstream-named directories here contain research overlays, not complete
upstream source checkouts. The standalone showcase is included directly; to
reconstruct the full development workspace:

```bash
python3 scripts/bootstrap_workspace.py --destination "$HOME/projects/rise-workspace"
cd "$HOME/projects/rise-workspace/BrickSim"
uv sync --locked
```

The bootstrap clones pinned upstream revisions, applies the saved patches,
copies the research overlays, and rebases historical checkpoint asset paths.
It records source/output hashes for those derived stage files. Original
checkpoint bytes and provenance remain unchanged in this repository.
Install the upstream projects' required system dependencies separately.

No virtual environments, credentials, drivers or physical robot connection
are stored here. The original lab assets and dependencies retain their upstream
licenses; see [third-party provenance](docs/THIRD_PARTY.md). This work is a
simulation experiment, not yet a calibrated physical digital twin.
