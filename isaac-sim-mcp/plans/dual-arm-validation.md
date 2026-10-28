# Lab two-arm experiment

Subsequent reload correction: see `shared-object-grasp.md`. The original 03
checkpoint passed live/composition checks but failed physics playback after a
fresh launch. Canonical second-rig joint frames and full-payload target
normalization are now fixed, and the repaired 04 checkpoint passed playback.

## Scope

Add a second complete lab UR5e/QC/RG2 rig to the existing scene. Preserve the
environment. After the first side-by-side trial, the user requested opposite
table ends: bases at (+0.75, -0.30, 0) and (-0.75, -0.30, 0) m, yaw 180 and
0 degrees respectively. This is provisional, not calibrated lab geometry.
Do not launch ROS or physical robot drivers. Leave two arms separated at home.

## Work

- [x] Inspect articulation roots, reference arcs, external weld relationships,
  and instance-proxy colliders.
- [x] Implement repeatable cloning with all rig relationship targets remapped.
- [x] Validate independent bidirectional motion on all 12 arm joints, including
  the stationary arm's response.
- [x] Approach a controlled arm-arm contact and verify actual PhysX collider
  pairs, dynamic response, separation after withdrawal, and finite states.
- [x] Capture camera output, scene overlay, configuration, and JSON results.
- [x] Update post-clone inventory and research notes with measurements and limits.

## Layout change and checkpoints

The first trial passed all 24 movement cases and upper-arm contact/withdrawal
at the earlier 0.65 m side placement. Those results remain in
`resources/dual-arm-validation/` and do not validate the new end placement.

The user explicitly requested USDA checkpoints at useful milestones. Save
stationary referenced stages and JSON/SHA256 provenance in the workspace's
`checkpoints/` directory before repositioning, after dynamically folding the
coupled rigs, after end placement, and after validated withdrawal. Anchor
relative references to the source layer; retain original robot/gripper assets.
Reopen stages for composition checks without replacing the live scene.

Fold through drives, snapshot both arm/gripper poses together, then restore
the coupled posed specs after stopping physics before moving bases. Do not
repeat the known arm-only initial-coordinate teleport error.

## Accepted opposite-end result

`resources/dual-arm-end-table-validation/report.json` records `pass: true`.
All 24 cases passed; maximum free-space error across the moving and stationary
arms was 0.0004998334 rad. With pan zero and both shoulder lifts commanded
to -0.50 rad, forearm-forearm contact produced 29 reports, peak raw impulse
0.925822 N*s, minimum reported separation -2.862e-6 m, and tracking deflections
0.006509 / 0.006554 rad. After withdrawal to shoulder -0.80 rad, errors were
7.749e-6 / 6.551e-5 rad with no residual inter-rig contact. All states were finite.

The latest validated checkpoint is
`../../checkpoints/20261027T071807.819020Z-03-end-table-validated.usda`.
It was reopened in an independent USD stage with all payloads loaded: no
composition errors, and both rigs, environment, and bench present. Live viewport
uses `/World/dual_arm_overview`. No source USD asset or original stage was saved
over, and no ROS/physical driver was started.

An attempted import reused Kit's cached prior script; it was detected from the
actual base poses/constants and rejected, then current source was compiled
directly. A checkpoint metadata call used an unavailable USD Stage API before
any relocation; corrected to prim-index `localErrors`. Rejected artifacts remain
clearly labeled. Neither attempt is counted as validation of the final layout.

## Acceptance

Free-space position error <0.001 rad after two simulation seconds; no inter-rig
contact at home. Contact must name colliders from both rigs, with measurable
tracking deviation/contact response rather than mere visual overlap. All states
must stay finite; stop the contact approach at first detected contact and
withdraw. Final home must regain <0.001 rad position error and no contact.
