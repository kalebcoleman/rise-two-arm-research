# Shared-bar load and speed limits

Keep bar geometry 650x35x60 mm, object friction, original arm gains, and 1.0 RG2
drive caps fixed. Reset to accepted grasp checkpoint 06 for every trial. Increase
mass with matching cuboid inertia, and scale movement time separately. Do not
move the object directly or add grasp joints. Stop each failed trajectory and
retain measurements; do not save failed trials as approved checkpoints.

Measure relative pad/object translational and rotational slip, four-finger
contact, table clearance, object-goal tracking, actual linear/angular velocity,
and unexpected inter-rig/environment contacts. Classify slip onset (>2 mm or
>1 degree), gross slip/control loss (>10 mm / 5 degrees, lost bilateral contact,
support collision, or object tracking outside bounds) separately. Bounds refer
to this simulated geometry/friction/controller setup, not physical RG2 ratings.

## Work

- [x] Explain local Content path and add human-readable Approved checkpoint copies.
- [x] Implement reproducible mass/speed sweep and measured boundary refinement.
- [x] Find mass and light-bar speed transition brackets; test their interaction.
- [x] Replay successful checkpoints, add short mass/speed names, and document
  measured limits, failure modes, and unresolved ranges.

## Accepted results

20 trials completed, 13 passed and 7 failed. Mass bracket at 1x: 1.25 kg passed,
1.5 kg failed relative grasp drift (4.869 mm against 2 mm); both repeated with
the same outcome. A 2 kg bar dropped to the ground-plane collider during
initialization at 0.25 simulation seconds. Light-bar speed bracket: 5x passed,
6x failed object-goal tracking (10.052 mm against 10 mm), both repeated. This
speed boundary is tracking, not evidence of first gripper slip.

Combined: 1.25 kg/2x and 0.625 kg/5x passed; 1.25 kg/5x failed tracking.
Peak measured speeds at the 0.1 kg/5x pass were 0.104 m/s and 17.70 degrees/s.
Eleven human-readable saved-stage copies passed strict replay: four fingers
contacting in all samples, no support contact, correct mass, gravity enabled,
kinematic false, <=0.18 mm replay drift. Failed conditions have no Approved USD.

Content browser was navigated to `checkpoints/Approved/`, with short milestone
names plus Heavier Bars/Faster Moves/Combined Loads folders. JSON notes were
separated into Test Notes folders; originals/checksums retained. Full results
and measured map: `resources/bar-load-speed-limits/run-001/` and
`docs/BAR_LOAD_SPEED_LIMITS.md`. Brackets are criterion-dependent simulated
observations, not physical ratings or validated safety margins.
