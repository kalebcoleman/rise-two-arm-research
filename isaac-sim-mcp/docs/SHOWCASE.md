# Showing the two-arm experiment

Validated October 28, 2026 after [repairing the GPU driver](GPU_REPAIR.md).
Fresh physics passed; the 26.07-second recording was reopened independently.
Play moved its bar 16.93 mm in the first two seconds, and start/middle/end
frames rendered and were inspected.

**Open `checkpoints/Approved/00 WATCH MOTION.usda` and press Play to watch now.**
Select `/World/dual_arm_overview` as the viewport camera if needed.
The source run kept four-finger contact throughout with zero unexpected
collisions, maximum slip 0.135 mm / 0.036 degrees, minimum clearance 99.5 mm,
and maximum tracking error 2.02 mm / 0.354 degrees. Reports, hashes and images
are in `showcase/recordings/`. This is recorded measured motion; the launcher
below runs fresh physics.

## What the files mean

| File | What Play does |
| --- | --- |
| `checkpoints/Approved/00 WATCH MOTION.usda` | Plays the recorded accepted trajectory. |
| Other `checkpoints/Approved/*.usda` | Starts physics holding the saved pose. These are experiment restore points. |
| `showcase/Start Physics.usda` opened alone | Also holds the start pose; it needs the demo launcher to command motion. |
| `showcase/recordings/WATCH - Two Arm Motion.usda` | Replays measured transforms on the timeline; created only after a passing demo run. Physics is disabled in this presentation copy. |

A held pose is not the motion history. Previously, Python commanded the arms
while the simulator was running; those Python processes were not stored in
the checkpoint. “Checkpoint replay passed” in older reports means the held
pose survived reopening and physics, not that the trajectory was replayed.

## First run on a working Isaac Sim workstation

Use Isaac Sim **5.1.0**, with a working NVIDIA GPU and its normal Python
dependencies (NumPy, SciPy and Pillow). From the transferred `isaac-sim-mcp`
directory:

```bash
RISE_PYTHON=/path/to/isaac-sim/python.sh ./scripts/run_showcase.sh
```

For the existing adjacent BrickSim virtual environment, run
`./scripts/run_showcase.sh` directly. The launcher resolves paths from its own
location; it does not require `/home/kaleb`, the MCP bridges, ROS, the original
room, or the local display-output name.

Press **Play**. The arms begin already grasping the 0.1 kg bar, then translate,
yaw, pitch, return and move to a final pose. This demo covers coordinated
manipulation, not the initial approach/grasp/release sequence. **Reset Demo**
reloads the start scene for another run. Do not switch stages during a run.

The launcher uses the existing object-space IK, position drives, gripper close
target and measured contact checks. It does not tune the controller or increase
grip strength. It requires at most 2 mm translation slip, 1 degree rotational
slip, at least 50 mm clearance, 10 mm/5 degree tracking bounds, at least 95%
four-finger contact, and no unexpected collisions.

After success, open `showcase/recordings/WATCH - Two Arm Motion.usda` in Isaac
Sim, choose `/World/dual_arm_overview` as the camera if needed, and press Play.
Keep its `Recorded Motion.usdc` and `materials` directory alongside it. Stop/rewind to replay. This file
shows the recorded simulation; use the launcher for a fresh physics test.

For an unattended recording and check:

```bash
RISE_PYTHON=/path/to/isaac-sim/python.sh ./scripts/run_showcase.sh --headless --record
```

Success returns exit code 0 and writes the USDA, `report.json`, segment images
and `replay-manifest.json`. A failed run writes failure details and does not
approve a new replay; check the report rather than trusting an old WATCH file
from a previous run. Recordings can be placed elsewhere with `--output PATH`.

## Portability and transfer

`showcase/scene.usdc` embeds the robot, grippers, quick changers and bench
geometry. `Start Physics.usda` is its readable entry layer. Keep both and the
`materials` folder together. This presentation variant replaces the original
room with a simple tabletop at z=0 and studio lights and removes the ROS graphs
and loose objects. This variant passed the fresh physics run described above;
earlier experiment reports still describe their original scene.

The generated transfer archive at `RISE/transfers/RISE-school-showcase.zip`
contains these files, the required controller/recording scripts and research
guides. Extract it and run the command above. It does not install Isaac Sim,
include virtual environments, transfer the entire lab asset collection, or
replace the complete research workspace backup.

The private `kalebcoleman/rise-two-arm-research` repository packages the demo,
research additions, checkpoints, evidence, pinned upstream revisions and
workspace reconstruction script. Clone it and open the WATCH file to view.
The full development setup is described in its README. Isaac Sim is installed
separately. The original upstream repositories alone do not contain these additions.

## Implementation and checks

- `scripts/build_showcase.py`: scratch copy of accepted checkpoint 06,
  presentation environment, flattened embedded geometry, relative materials.
- `scripts/launch_showcase.py`: Play-triggered controller, measured transform
  capture, passing-run export with physics schemas removed, reopened timeline.
- `scripts/coordinated_bar_motion.py`: optional per-update sample callback and
  `save_checkpoints=False`; default research checkpoint behavior is unchanged.
- Offline checks: Python compilation, shell syntax, relocated stage composition,
  dynamic object flags, no absolute home paths, synthetic transform samples and
  removal of rigid-body physics in the exported synthetic recording.
- Fresh GPU physics, independent exported replay Play, rendered frames and
  visual inspection passed. The pinned-upstream workspace reconstruction passed.
- `scripts/verify_motion_replay.py`: independent playback/hash/render check;
  add `--show` to leave a visible viewer ready at time zero.
