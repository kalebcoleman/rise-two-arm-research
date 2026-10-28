# Shared-object grasp and lift

The subsequent [coordinated-motion experiment](COORDINATED_BAR_MOTION.md)
translated and rotated this grasp without releasing the object. Checkpoint 08
is the newest accepted moved pose; checkpoint 06 below remains the accepted
initial grasp/lift baseline.

Accepted on 2026-10-27 (America/Phoenix), Isaac Sim 5.1.0, using the two lab
UR5e/QC/RG2 rigs at opposite table ends. The simulator was reopened at the
user's request; no physical robot driver was started.

## What was added

- `scripts/shared_object_experiment.py`: USD-joint-frame forward kinematics,
  numerical pose IK, coordinated approach/lift trajectories, independent RG2
  commands, actual contact tracking, slip/clearance tests, gravity release, and
  checkpoint playback verification.
- `/World/SharedBar`: a 650 x 35 x 60 mm dynamic box, 0.1 kg, gravity enabled,
  static friction 0.8, dynamic friction 0.6, restitution zero. The mass and
  material values are provisional simulation parameters, not lab calibration.
- Top-down grasps near x=+/-0.22 m, y=-0.30 m. The open-pad grasp datum is at
  z=0.085 m, then moves up 0.10 m. Both grippers close through their actuated
  joints to the existing -0.45 rad close target and original 1.0 drive cap;
  mimic joints follow mechanically. No gains or force caps were raised.
- The USD and tensor gripper targets are kept consistent, including release.
- Checkpoint support now preserves RG2 closing targets when saving a loaded
  grasp, while normalizing other targets/velocities in the saved layer.

The object is held through physical contact and friction. There are no object
joint connections, grasp welds, hidden supports, kinematic object motion,
disabled gravity, or object pose updates during lift.

## Accepted measurements

`../resources/shared-object-validation/trial-004/report.json` contains the full
trace, planned poses, collider events, and pass criteria. The test first closes
both grippers on the supported bar, lifts, holds for two simulation seconds,
opens both grippers, then reloads the held-stage checkpoint.

| Check | Result |
| --- | --- |
| Both left/right fingers contact at each gripper | Passed |
| Minimum object bottom clearance during hold | 99.388 mm |
| Maximum relative object/gripper slip | 1.718 mm |
| Hold samples with all four finger contacts | 100% |
| Gravity-driven drop when both grippers open | 99.522 mm |
| Grasp retained after held-stage reload | Passed |
| Exact final checkpoint replay, additional two seconds | Passed; minimum clearance 99.495 mm |
| Object gravity / kinematic flag / object joints | Enabled / false / none |

Acceptance was at least 50 mm clearance, at most 10 mm slip, and all four
finger contacts in at least 80% of hold samples. Actual opposing finger
collider paths are recorded, rather than inferring grasp success from images.

The measured FK pad position errors were below 0.3 micrometers before planning;
this verifies the implementation against the simulator's USD geometry, not
against a physical robot. Contact force/load sharing, payload margins, other
objects, and robustness across repeated randomized trials remain unvalidated.

## Accepted checkpoint and evidence

Latest accepted file in the workspace `checkpoints/` directory:

`20261027T162819.114595Z-06-shared-grasp-lift-validated.usda`

Its JSON sidecar records SHA256 and composition metadata. The exact file was
reopened and physically played for two more simulation seconds with all four
articulations finite, both finger pairs contacting, and the object still held.
Evidence is `trial-004/checkpoint-playback.json` and
`trial-004/reloaded-validated.png`. The live simulator is left on that checkpoint
with the object held and the overview camera selected.

The checkpoint preserves force-limited gripper closing targets. Freezing those
targets at the stalled contact angle would remove commanded grip preload.
Files remain local referenced USDA stages, not portable asset bundles or PhysX
solver-cache/process snapshots. See `../../checkpoints/README.md`.

## Failed attempts and fixes retained

- The former `03-end-table-validated` stage passed live and composition tests
  but failed fresh physics playback. Posed body-relationship remapping had
  altered second-rig joint frames; restoring 16 canonical definitions fixed
  it. `restore_second_rig_joint_frames` now preserves this repair in source.
  Checkpoint normalization also changed from LoadNone to LoadAll so payload-
  defined joint types are processed. Repaired baseline 04 passed actual
  reload/playback. The old 03 file is retained and marked superseded.
- Trial 001: grippers touched the bar with linkage/truss arms instead of finger
  pads. No lift was accepted. Raised the grasp datum by 45 mm so the fingertips
  engage while linkage geometry clears the bar.
- Trial 002: initialization pushed/rotated the bar before approach. Preparation
  now clears both open grippers above the table before resetting the object,
  and preserves current initial arm targets when reopening the setup stage.
- Trial 003: four-finger grasp, lift, and hold passed, but release failed because
  a queued USD close target overwrote a tensor-only open command. Synchronizing
  the two target representations fixed release; trial 004 reran the full test.

Reports and images for trials 001–003 remain alongside the accepted run. Files
labeled pending are milestones, not the final accepted resume point.

## Repeating

Start from the repaired 04 baseline or accepted 06 grasp checkpoint, with the
simulation playing. Compile the current source directly inside the simulator
through MCP (Kit's cached importer previously reused stale code). Load
`dual_arm_experiment.py` as the checkpoint-support module, then load
`shared_object_experiment.py`. Schedule a coroutine that awaits
`inspect_and_plan(output_root)` followed by
`run(output_root / 'trial-name', checkpoint_support)`. The planning file must
be in the trial directory's parent. Use a new trial name to preserve evidence.

Watch the module's `STATE['phase']` and await its task before reading results.
These APIs depend on Isaac Sim and cannot run under plain system Python.
