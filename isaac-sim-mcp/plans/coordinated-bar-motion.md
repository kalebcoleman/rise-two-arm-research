# Coordinated bar translation and rotation

Start from accepted grasp checkpoint 06 with gravity enabled and no object
joint/kinematic attachment. Keep both original force-limited gripper commands.
Do not start ROS or physical drivers. Preserve the accepted input checkpoint.

## Motion and acceptance

Test slow shared-object targets with up to 50 mm translation, 30 mm lift,
10-degree yaw, and 5-degree pitch, plus a return to the initial pose. End at a
translated/rotated pose for a new milestone. Derive both gripper goals from the
same object transform and initial object/gripper transforms, then solve native
USD-frame IK independently. Do not move the object directly.

- Object/gripper translation slip <=10 mm and rotation slip <=3 degrees.
- At least 95% of sampled states retain both finger contacts on both grippers.
- Minimum bar/table clearance >=50 mm and object pose errors <=10 mm / 5 degrees.
- No inter-arm contact or non-mount robot/environment collision. Object contacts
  must be finger contacts; no ground/table/linkage support during movement.
- All states finite. Stop advancing the trajectory if limits are exceeded.
- Save a checkpoint only on success, then replay that exact USDA and verify
  that both grippers retain the bar with physical contacts.

## Work

- [x] Verify simulator is available and checkpoint 06 is loaded/playing.
- [x] Implement trajectory, slip, pose, contact, and collision measurements.
- [x] Execute coordinated translation/yaw/pitch/return and final pose.
- [x] Replay the new checkpoint and document results and limitations.

## Accepted result

Trial 001 passed without retries. 449 samples over 32.917 simulation seconds:
maximum motion slip 0.08246 mm / 0.02451 degrees; four-finger contact 100%;
minimum clearance 99.486 mm; object tracking errors 1.155 mm / 0.2016 degrees;
zero unexpected contacts. The intermediate held-pose replay also passed, with
maximum relative slip 0.1217 mm / 0.03205 degrees from the original reference.

Final checkpoint:
`../../checkpoints/20261027T164045.283144Z-08-coordinated-bar-motion-validated.usda`.
Exact-file replay passed for two additional seconds: four finger contacts,
124.411 mm minimum clearance, no unexpected collisions, gravity enabled,
kinematic false, no object joints. Results and per-segment images are in
`resources/coordinated-bar-validation/trial-001/`. Simulator remains playing the
new held pose. Syntax, whitespace, and checkpoint checksum checks accompany
the physical tests. Existing self-collision/fixture exclusions and uncalibrated
physics are documented in `docs/COORDINATED_BAR_MOTION.md`.
