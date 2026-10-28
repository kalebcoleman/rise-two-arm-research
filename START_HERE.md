# Understanding the two-arm project

Start with [the showcase guide](isaac-sim-mcp/docs/SHOWCASE.md). Existing USDA
checkpoints are held-pose restore points. A motion recording requires sampled
transforms or a running controller. After the October 28 GPU repair, the fresh
physics run and independent recorded Play/render checks passed. Open
`checkpoints/Approved/00 WATCH MOTION.usda` and press Play to watch.

## What we have built

The lab repositories supplied the UR5e, RG2, quick changer, Vention CAD and
integration tools. Local work added collision setup and validation, corrected
shoulder tracking and second-rig joint frames, arranged two arms across the
bench, introduced a shared dynamic bar, and tested grasp/lift and coordinated
motion. [POST_CLONE_CHANGES.md](POST_CLONE_CHANGES.md) separates supplied assets
from additions and links the detailed evidence.

Read the experiments in this order:

1. [Two arms and collision validation](isaac-sim-mcp/docs/DUAL_ARM_EXPERIMENT.md).
   Understand link frames, joint targets and why saved joint frames matter.
2. [Shared-object grasp](isaac-sim-mcp/docs/SHARED_OBJECT_EXPERIMENT.md).
   Understand the pad frames, finger contacts, gravity release and slip metric.
3. [Coordinated motion](isaac-sim-mcp/docs/COORDINATED_BAR_MOTION.md).
   Understand object-space goals, per-arm IK, position control and measured
   object response. The bar is dynamic; it is not commanded along the path.
4. [Mass/speed limits](isaac-sim-mcp/docs/BAR_LOAD_SPEED_LIMITS.md).
   Separate grip drift/drop from arm tracking failure. At baseline speed,
   1.25 kg passed and 1.5 kg exceeded the drift bound; a 0.1 kg bar passed 5x
   and exceeded tracking bounds at 6x. These are tested conditions in this
   model, not robot payload ratings or a general safe envelope.

## Next phases, in your requested order

First view the verified demo after transfer, then walk through
one trajectory and its report together. Pause controller changes until the
coordinate frames, controller inputs, measurement outputs and pass/fail rules
are understandable and reproducible.

Then establish a repeatable baseline and change one controller or task variable
at a time. Preserve the existing controller, log joint/target motion, object
pose, contacts and saturation, and repeat trials around the observed failure
boundaries before attempting heavier or more complex tasks.

For a digital twin, first measure the actual workstation: base/tool/camera
transforms, joint offsets, object mass/inertia, actuator and gripper behavior,
latency, and contact properties. The current model has uncalibrated contacts,
position drives and robot gravity suppression; it is not yet a calibrated twin.

For sim-to-real research, define measurable discrepancies and a reproducible
task protocol, collect paired simulated and real observations, calibrate using
one dataset and evaluate on held-out conditions. Candidate questions include
whether contact/actuation uncertainty predicts slip and whether controller
improvements transfer across masses and speeds. Real-hardware commissioning
and experiment procedures are a separate future phase; no hardware driver is
started by this showcase.
