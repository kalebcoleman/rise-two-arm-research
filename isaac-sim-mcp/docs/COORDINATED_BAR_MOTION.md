# Coordinated bar translation and rotation

Accepted on 2026-10-27 (America/Phoenix), Isaac Sim 5.1.0. Starts from shared
grasp checkpoint 06 and keeps the same 0.1 kg bar, friction parameters, robot
gains, RG2 closing commands, and gravity settings. No physical driver is used.

## Implementation

`scripts/coordinated_bar_motion.py` measures the initial object/gripper
transforms, then derives both gripper targets from a shared object-space goal.
Independent USD-frame IK solves each six-joint arm target. Smooth interpolated
targets are sent to the native articulation controllers. The bar is measured
through its rigid-body tensor pose and is never commanded or teleported.

The grippers retain their original force-limited -0.45 rad closing commands.
There is no object weld, kinematic object, hidden support, disabled object
gravity, or gain increase. Robot-chain gravity stays disabled as in the supplied
position-mirroring rig. This is position-based coordination, not a calibrated
load-sharing or force controller.

## Tested trajectory

Offsets and rotations are relative to the initial held bar, in world axes.
Rotation is composed as Rz(yaw) Ry(pitch) followed by the initial orientation.

| Segment | Translation offset, meters | Yaw | Pitch |
| --- | --- | --- | --- |
| Translate | `(0, +0.050, +0.030)` | 0 degrees | 0 degrees |
| Yaw | same offset | +10 degrees | 0 degrees |
| Pitch | same offset | +10 degrees | +5 degrees |
| Return | `(0, 0, 0)` | 0 degrees | 0 degrees |
| Final | `(+0.025, +0.025, +0.025)` | -10 degrees | 0 degrees |

The final pose was held for two additional simulation seconds. Including
segment settling, the measured trajectory spans 32.917 simulation seconds and
449 sampled states. Contact events were also monitored between samples.

## Results

Evidence: `../resources/coordinated-bar-validation/trial-001/report.json`.
No trial failures or retries were required for this trajectory.

| Measurement during motion/hold | Observed | Acceptance |
| --- | --- | --- |
| Maximum relative translational slip, either gripper | 0.0825 mm | <=10 mm |
| Maximum relative rotational slip | 0.02451 degrees | <=3 degrees |
| Both fingers of both grippers contacting | 100% of samples | >=95% |
| Minimum object/table clearance | 99.486 mm | >=50 mm |
| Maximum object position error from moving goal | 1.155 mm | <=10 mm |
| Maximum object rotation error from moving goal | 0.2016 degrees | <=5 degrees |
| Unexpected collision events | 0 | 0 |

Slip is measured as change in the object pose relative to each measured pad
frame. Object tracking error is a separate measure against the moving world
goal; it includes controller lag and must not be described as grasp slip.

Inter-rig collisions, non-mount robot/environment collisions, and bar contacts
with anything except finger links trigger failure. Fixed-base mounting contacts
and mechanical contacts within the same existing rig are excluded. The supplied
UR5e's self-collision policy is unchanged; this is not a validation of disabled
self-collision pairs or a general collision-avoidance planner. Contact coverage
is sampled as stated above, not a claim that every physics substep had four
contacts. All sampled states stayed finite.

## Checkpoint and replay

The accepted final pose is saved as:

`../../checkpoints/20261027T164045.283144Z-08-coordinated-bar-motion-validated.usda`

Its JSON sidecar records the SHA256 and 24 normalized angular states. Gripper
closing preload remains enabled in the snapshot. The prior 06 grasp checkpoint
was preserved; 07 is an intermediate pending-replay milestone.

The first held-pose replay retained four finger contacts, with maximum relative
translation slip 0.122 mm and rotation slip 0.0321 degrees from the original
grasp reference. The **exact final 08 file** was separately reopened and played
for two more simulation seconds: 124.411 mm minimum clearance, four finger
contacts, zero unexpected contacts, gravity enabled, kinematic false, and no
object joint connections. See `trial-001/checkpoint-playback.json` and
`trial-001/reloaded-validated.png`. The live simulator is left on that final
checkpoint in its translated/rotated held pose.

These USDA snapshots reference local geometry assets. They preserve the scene
and targets, not solver caches or running Python/controller processes.

## Repeating and limits

Load and play an accepted shared-grasp checkpoint. Compile the current sources
directly into simulator Python modules to avoid Kit's stale import cache:
`shared_object_experiment.py` for FK/IK/pad frames, `dual_arm_experiment.py` for
checkpoint support, and `coordinated_bar_motion.py` for this test. Schedule
`motion.run(new_output_directory, shared_module, checkpoint_module)` on the Kit
async loop. Inspect `STATE` and await task completion before reading results.
Use a fresh output directory to retain prior evidence.

The pass covers this slow deterministic trajectory and parameter set. Friction
calibration, force/load sharing, heavier or different objects, faster motions,
randomized robustness, physical robots, and arbitrary collision-free paths are
not established by this result.
