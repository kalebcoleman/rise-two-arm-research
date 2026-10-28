# Shared-object bimanual grasp and lift

## Scope and acceptance

Use the opposite-end lab rigs and preserve the prior validated checkpoint.
Add one shared rigid object with gravity, realistic collision geometry, and
documented provisional mass/material. No fixed grasp joints, hidden supports,
object teleport during lift, disabled object gravity, or physical robot driver.

A successful trial must establish opposing finger contacts on both grippers,
lift the object at least 50 mm off its original support, keep it clear for two
simulation seconds, and bound relative object/gripper slip. Opening either
gripper or the final release must be observable through physical response.
Save a validated USDA checkpoint only on successful physical grasp/lift.
Retain rejected trials and do not substitute visual overlap for retention.

## Work

- [x] Confirm checkpoint directory and USDA format to the user.
- [x] Reopen simulator and the last validated stage (both MCP endpoints were
  initially unavailable; user requested help reopening).
- [x] Verify independent gripper opening/closing and pad frames.
- [x] Derive and verify USD-based forward kinematics, then solve
  approach/grasp/lift configurations numerically.
- [x] Add shared object, close both force-limited grippers, and measure contact,
  clearance, retention, and relative slip during coordinated lift and hold.
- [x] Document measured success/failure; save accepted checkpoint if criteria pass.

## Reload defect found before grasping

Opening the prior checkpoint and actuating grippers exposed non-finite states
in the second rig. Live motion/collision and USD composition checks had passed,
but physics playback after reload had not been tested. The user supplied a
screenshot of the same broadphase error; simulation was stopped immediately.

Remapping posed joint body relationships had changed the second rig's local
joint frames. Restoring all 16 joint definitions from the canonical first rig
fixed playback. The checkpoint normalizer also used LoadNone, skipping joint
types supplied by payloads; changed to LoadAll and checked 24 normalized joints.
New baseline `20261027T161920.485684Z-04-joint-frames-repaired.usda` passed actual
reload/playback with both arms and both grippers finite. Original 03 file is
retained but superseded. Do not equate composition checks with dynamic reload.

FK from canonical USD joint frames matches measured pad frames within 0.3 um
and 0.4 urad. Both 0.78 rad gripper opening commands track independently.
Top-down targets were solved numerically for both ends of a 650x35x60 mm bar.

## Trial 001 (rejected)

0.1 kg bar, friction 0.8 static / 0.6 dynamic, gravity enabled. Initial pad
target z=0.040 m placed the truss links alongside the bar. Closing stalled near
0.231 rad with both left/right truss arms contacting, but neither finger link
contacting. Stopped before lift and saved the failed report/images in
`resources/shared-object-validation/trial-001/`. This is not a successful grasp.

Measured finger bounds were z=0.0004..0.0593 m and truss bounds
z=0.0242..0.0912 m. Raising the grasp by 45 mm is intended to clear the trusses
above the 60 mm bar while engaging the fingertips; trial 002 tests that change.

## Trials 002 and 003

Trial 002 was rejected because the bar was pushed/rotated during initialization
before the approach, then missed by the grasps. Preparation now moves the open
grippers to a clear pregrasp before resetting the bar and authors arm targets
to the current pose before reloading the setup stage.

Trial 003 established all four finger contacts and lifted the bar. Minimum
two-second hold clearance was 99.066 mm, maximum relative slip 1.993 mm,
bilateral contact fraction 100%. No grasp joint, hidden support, or object
gravity change was used. The release test exposed a command-order defect:
an authored USD closing target was applied after the tensor-only open command.
USD and tensor gripper targets are now kept consistent. Trial 004 reruns the
complete suite including gravity release and checkpoint playback.

## Accepted trial 004

Minimum hold clearance 99.388 mm; maximum relative slip 1.718 mm; both finger
pairs contacting in 100% of two-second hold samples. Both-open release produced
a 99.522 mm gravity drop. Held checkpoint reload retained the grasp. Exact final
06 checkpoint playback also passed for two seconds, with 99.495 mm minimum
clearance, four finger contacts, gravity enabled, object kinematic false, and
no object joint connections. No physics substitution or grasp weld was used.

Accepted checkpoint:
`../../checkpoints/20261027T162819.114595Z-06-shared-grasp-lift-validated.usda`.
Reports, event traces, and images are in
`resources/shared-object-validation/trial-004/`. Scene is left playing in the
successful held pose. Broader robustness, calibrated friction/force/load sharing,
and physical-robot operation remain outside this acceptance result.
