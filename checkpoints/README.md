# Stage checkpoints

For browsing by short human-readable names, open
`/home/kaleb/projects/RISE/checkpoints/Approved/` in Content. That folder contains
only copies of accepted stages: Empty Hands, Grasp and Lift, and Move and Rotate.
Timestamped originals and their JSON test provenance remain in this parent folder.

The new `Approved/Heavier Bars`, `Approved/Faster Moves`, and
`Approved/Combined Loads` folders hold successful stress-test scenes, with names
such as `1.25kg - 2x.usda`. All eleven short-named load-test stages passed strict
physical replay. See `../isaac-sim-mcp/docs/BAR_LOAD_SPEED_LIMITS.md` for the
measured pass/failure brackets and interpretation of speed versus grip failure.

## Latest accepted coordinated motion

[20261027T164045.283144Z-08-coordinated-bar-motion-validated.usda](20261027T164045.283144Z-08-coordinated-bar-motion-validated.usda)
is the current resume point. Translation, yaw/pitch rotation, return, and final
pose passed slip/contact/collision checks. Exact-file physics playback also
passed. The object remains physically held at a translated/rotated pose.
See `../isaac-sim-mcp/docs/COORDINATED_BAR_MOTION.md` and
`../isaac-sim-mcp/resources/coordinated-bar-validation/trial-001/checkpoint-playback.json`.
The 07 file is its intermediate pending-replay milestone.

## Latest accepted shared-object hold

[20261027T162819.114595Z-06-shared-grasp-lift-validated.usda](20261027T162819.114595Z-06-shared-grasp-lift-validated.usda)
is the prior accepted shared-grasp baseline. Both grippers physically hold the shared
bar about 99 mm above the table. Lift, two-second retention, gravity-driven
release, and actual replay of this exact file passed. See
`../isaac-sim-mcp/resources/shared-object-validation/trial-004/checkpoint-playback.json`.
The adjacent JSON contains its SHA256. Closing commands remain force-limited
and preserved for grasp preload; no object weld or disabled gravity is used.

The 04 file below is the repaired empty-hand baseline. Earlier 01–03 snapshots
are historical, not dynamically validated resume points. Files labeled
05/pending preserve a hold before all subsequent acceptance checks completed.

**Reload correction:** the original `03-end-table-validated` file passed live
tests and composition reopening but failed subsequent physics playback. Do not
use it as a resume point. The repaired baseline is
`20261027T161920.485684Z-04-joint-frames-repaired.usda`; actual reload/playback
passed for both arms and both grippers (`04-playback-validation.json`). The
shared-object experiment may produce a newer accepted grasp checkpoint below.

USDA checkpoints are saved at useful experiment milestones. Filenames carry a
UTC timestamp (`Z`) and a phase label; project notes use America/Phoenix time.
Each complete checkpoint has an adjacent JSON file containing its purpose,
source stage, SHA256 checksum, composition check, and presence of both arms.

Current milestones:

1. `01-before-end-table-layout`: prior stable two-arm placement.
2. `01b-folded-before-placement`: complete welded rigs folded dynamically before
   changing base positions.
3. `02-end-table-setup`: opposite table ends, separated folded home; tests pending.
4. `03-end-table-validated`: movement, contact, withdrawal, and recovery accepted.

Choose the latest **validated** checkpoint when resuming the experiment. Open
it through Isaac Sim's stage-open workflow; preserve the current scene first.
The files include the two robots, grippers, bench collision proxies, cameras,
and environment references. Each snapshot zeros velocities and holds current
joint positions so opening it does not intentionally replay the contact motion.

Historical opposite-end checkpoint, superseded by the repair above:
[20261027T071807.819020Z-03-end-table-validated.usda](20261027T071807.819020Z-03-end-table-validated.usda).
The final file was reopened with all payloads loaded and no composition errors;
`validated-reopen-check.json` records that inspection. Earlier accepted milestone
timestamps are `071658.412010Z`, `071701.492917Z`, and `071705.550706Z` on the
same UTC date. The validated scene passed 24 movement cases and forearm contact
followed by successful withdrawal.

These are local referenced stages, not portable asset bundles. Keep the source
robot/gripper/workstation assets and environment files at their documented
paths. They do not serialize PhysX solver caches, controller Python handles,
running processes, ROS drivers, or the history of a simulation. Reacquire
articulation handles after opening a checkpoint. A checkpoint does not validate
unmeasured physical calibration, friction, or force accuracy.

Files named `incomplete-*` are rejected diagnostic artifacts, not resume points.
The initial metadata-API attempt is retained under that name; it was replaced
by checkpoints checked through the installed USD prim-index API.

The repeatable creator is
`../isaac-sim-mcp/scripts/dual_arm_experiment.py:save_checkpoint`. It copies and
normalizes a separate snapshot layer and does not save over the source stage.
