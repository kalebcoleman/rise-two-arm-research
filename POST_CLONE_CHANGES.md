# RISE local changes after cloning

October 28 addition: [portable demo and recording workflow](isaac-sim-mcp/docs/SHOWCASE.md),
with an embedded presentation scene and [understanding guide](START_HERE.md).
The GPU was repaired by installing the missing module for kernel 7.0.0-38 and
matching NVIDIA 580.178.04 components. Fresh physics, exported Play and rendered
replay checks passed; see `checkpoints/Approved/00 WATCH MOTION.usda`.
Other checkpoints remain saved poses. The private `rise-two-arm-research`
repository preserves the additions, evidence and upstream reconstruction.

Current accepted stage: [coordinated motion checkpoint 08](checkpoints/20261027T164045.283144Z-08-coordinated-bar-motion-validated.usda).
The [coordinated-motion record](isaac-sim-mcp/docs/COORDINATED_BAR_MOTION.md) includes
translation/rotation, slip/contact/collision, and exact-checkpoint playback results.
For human-readable names and the newer load/speed cases, browse
`checkpoints/Approved/` and its condition folders. See the
[load/speed limits](isaac-sim-mcp/docs/BAR_LOAD_SPEED_LIMITS.md).

Dates use America/Phoenix. This inventory covers verified source changes, local
setup, and research artifacts. It distinguishes supplied upstream features from
local work; it does not claim that an imported model is a calibrated digital twin.

## Clone baselines

Git reflogs identify these original checkout revisions. No local commits have
replaced them as of this work; review working-tree diffs and untracked files.

| Checkout | Origin | Clone revision |
| --- | --- | --- |
| `BrickSim` | `intelligent-control-lab/BrickSim` | `cbd5de3` |
| `isaac-sim-mcp` | `inbarajaldrin/isaac-sim-mcp` | `27f610056017d7bffc2ff56fac6d219702b781a0` |
| `omni-kit-mcp` | `inbarajaldrin/omni-kit-mcp` | `68f738f60056a02701f449bfe043f676df35210e` |

The lab UR5e, RG2, quick changer, Vention CAD, cameras, FMB assets, robot-chain
gravity suppression, recording, and reusable Kit MCP bridge were supplied by
the lab repositories. They were **not created locally after cloning**. The
generic bridge checkout currently has no tracked source modifications.

## Workstation and dependencies

- Local dependency changes in `BrickSim/pyproject.toml` and `uv.lock`: CUDA 12.8
  PyTorch packages for the RTX 5080 and OpenCV 4.11.0.86 for NumPy 1.x.
  The environment uses Isaac Sim 5.1.0 and Isaac Lab 2.3.2.post1. No system GPU
  driver changes were needed. The native extension is an upstream prebuilt
  binary, not a locally developed custom physics solver.
- `BrickSim/docs/UBUNTU_SETUP.md`, assembly validation logs, and a desktop image
  record installation checks and the four-step assembly test.
- `BrickSim/scripts/run_desktop.sh` selects this machine's usable primary
  display to avoid a GLFW/Vulkan work-area failure, then launches the selected
  scene. This display choice is machine-specific.
- Existing untracked `BrickSim/run_teleop.py` was preserved. Its authorship and
  validation were not established by this work.

## Research scaffolding

- `BrickSim/experiments/rise_dual_ur5e.py`: separate generic two-arm scene with
  parent-Xform placement, configuration validation, bounded-state checks,
  short simulation recording, and metadata capture. It does not grasp objects
  or coordinate two-arm manipulation.
- `BrickSim/research/configs/dual_ur5e.json`: provisional base poses, joint
  targets, shared object, timestep, mass, and friction settings.
- `BrickSim/experiments/tests/`: four invalid-configuration checks.
- `BrickSim/research/README.md`, `PROTOCOL.md`, `NOTEBOOK.md`, `MCP_SETUP.md`, and
  `workspace/`: provenance, experiment workflow, dated results, workspace
  templates, and scene audit. Generated run directories retain rejected as
  well as accepted baselines, configuration hashes, state traces, manifests,
  and saved stages. They are not self-contained asset bundles.
- The accepted generic two-arm smoke test is documented in the notebook:
  `outputs/rise/20261027T061121.266491Z/`. It remains separate from the lab's
  original single-arm cell. The subsequent lab two-arm experiment is described below.

## MCP integration and compatibility

- `BrickSim/scripts/run_rise_lab.sh`: launch the existing lab extensions with
  the simulator's bundled ROS libraries and local discovery configuration.
- `BrickSim/scripts/rise_isaac_mcp.sh`: stdio gateway wrapper selecting the lab
  endpoint and run-output directory.
- Local changes in `isaac-sim-mcp/exts/ur5e-dt/ur5e_dt/extension.py` before the
  collision work: owner-scoped `ur5e` tool registration, correct Python World
  initialization when a USD PhysicsScene already exists, and delegated
  `LogRedirector.isatty()` for logging compatibility.
- `isaac-sim-mcp/isaac_mcp/server.py`: normalize `ur5e.` before checkpoint
  gates, metadata categorization, and output-directory policy. Existing gate
  behavior is preserved.
- `isaac-sim-mcp/uv.lock`: records the editable adjacent `kit-mcp` dependency
  and resolved dependency metadata. Preserve both adjacent repository paths.
- Local Codex configuration registers `rise-kit` and `rise-isaac`. Simulator
  extension search paths/autoload were registered in the per-user Kit config;
  the prior config backup is identified in `research/MCP_SETUP.md`.
- Direct tool calls in the fresh local chat successfully verified Isaac Sim
  5.1.0, the endpoint on localhost:8766, and the five expected scene prims.
  This is stronger evidence than tool discovery alone.
- The optional ChatGPT tunnel route (client binaries, profile and
  `BrickSim/scripts/start_chatgpt_tunnel.sh`) was removed on 2026-10-28; local
  stdio MCP gateways are the only supported transport.

## Single-arm collision and tracking changes

Source is in `isaac-sim-mcp`:

- `exts/ur5e-dt/ur5e_dt/physics_setup.py`: one static oriented-box collision
  proxy per Vention CAD mesh (385 parts). Each proxy preserves the mesh's local
  bounds and world orientation. Physics material placeholders are static
  friction 0.6, dynamic friction 0.4, restitution 0; contact offset 1 mm and
  rest offset 0. Boxes approximate holes and concavities. The original roughly
  4.9-million-vertex render geometry and source USDs are unchanged.
- `extension.py`: the workstation loader installs/updates these proxies even
  when the bench already exists, restarting physics when necessary. Callers
  must reacquire tensor views after stage/physics changes.
- `physics_setup.py` and `extension.py`: raised shoulder target of -0.30 rad
  for initialization through drives. The asset's initial zero joint state is
  preserved to keep the separate gripper articulation and welds consistent.
  Drive gains and torque limits are not increased to overpower collisions.
- `scripts/validate_workstation.py`: repeatable live-simulator tests of both
  command directions for all six joints using simulation time, color/depth,
  bench-only probe contact, robot probe contact, cleanup, and control recovery.
  It temporarily disables the legacy ground/table colliders to isolate the
  bench, then restores them. No ROS or physical driver is started.
- `plans/collision-validation.md`: work log, correction of the initial audit,
  failure evidence, and final test status.
- Results belong in `resources/workstation-validation/`: JSON measurements
  and a camera frame. Consult `report.json` for actual pass/fail status;
  source installation alone does not establish successful validation.

### Corrections and failures retained

The initial audit incorrectly said the UR5e lacked colliders. Instance-aware
USD traversal finds seven enabled convex-decomposition link colliders. They
are preserved rather than duplicated. Initial camera and surface-drop tests
passed, but that drop exercised the legacy z=0 surface, not the Vention CAD.

The +0.03 rad shoulder command from the flat zero pose produced about 0.0051 rad
tracking error. PhysX contact reports identified wrist/quick-changer contact
with the legacy plane/table. Negative shoulder commands tracked within a few
microradians after three simulation seconds. This is a clearance issue, not
evidence for raising drive gains.

An intermediate attempt changed only the arm's initial joint coordinates,
leaving the separately articulated gripper at its original welded pose. The
combined test produced non-finite states and PhysX broadphase errors and was
stopped. The implementation now retains consistent initial coordinates and
moves to clearance with dynamic drives. This failed attempt must not be cited
as a successful physics test.

### Limits before two-arm research

The Vention panel's actual upper surface in the current placement is around
z=-0.085475 m; aligning the entire CAD assembly's highest point to z=0 does
not align that panel to z=0. The legacy table/plane still supplies the ordinary
work surface. Bench-only tests explicitly remove that confound temporarily.
Before using the Vention as the sole experimental table, calibrate its panel,
robot mounting position, and physical dimensions together.

Robot-chain gravity remains disabled by the supplied position-mirroring model.
Collision boxes, placeholder friction, camera depth, and joint tracking do not
validate grasp retention, contact force accuracy, ROS image publication,
physical calibration, or cooperative manipulation. No physical driver was
started. The single-arm run preceded the two-arm addition below.

## Lab second-arm addition (2026-10-27)

`isaac-sim-mcp/scripts/dual_arm_experiment.py` clones the complete existing lab
UR5e, quick changer, and RG2 into `/World/Arm2`. The user subsequently requested
opposite table ends: first base `(0.75, -0.30, 0)` m with yaw 180 degrees,
second base `(-0.75, -0.30, 0)` m with yaw 0 degrees. The bases are 1.5 m apart
and face inward. Home shoulder lift is -0.80 rad for initial clearance.
These poses are a provisional simulation layout; mounting locations are not
calibrated against the lab. The earlier 0.65 m side layout and its successful
measurements remain separately archived.

The script copies authored USD specs/reference arcs instead of flattening all
CAD geometry. Cross-root joint and material relationship targets are remapped
into `/World/Arm2`, and the script rejects joints that still target the first
rig. Instance colliders remain separate physical shapes despite shared geometry.
Both arms retain the original gains, fixed bases, and disabled chain gravity.
The second rig has its own six-joint articulation and gripper articulation. It
has no copied ROS subscription graph; movement uses independent simulator
articulation handles. No physical driver is launched.

The validation exercises both directions of every arm joint while checking the
other arm's hold error, approaches inter-rig contact gradually, records actual
PhysX collider pairs and raw solver impulses, and withdraws to separated home
targets. Reports, camera frames, and a referenced scene overlay are written to
`isaac-sim-mcp/resources/dual-arm-end-table-validation/`. Earlier-layout results
remain in `resources/dual-arm-validation/`. The source script reuses a matching
second rig, rebuilding it when the layout changes. The overlay depends on the existing
asset tree and is not a portable flattened bundle.

This is a collision/movement baseline, not coordinated shared-object grasping,
calibrated force control, or proof of collision avoidance for arbitrary paths.
See `isaac-sim-mcp/plans/dual-arm-validation.md` and `report.json` for the
measured validation outcome.

The opposite-end rerun passed all 24 movement cases with worst error
0.000499834 rad (bound 0.001 rad). Forearm-forearm contact occurred at both
shoulder commands -0.50 rad, with nonzero solver impulses and approximately
0.0065 rad contact deflection. Withdrawal to -0.80 rad left no inter-rig contact;
return errors were 0.00000775 and 0.00006551 rad. Both complete rigs remained
finite. See the [experiment and checkpoint guide](isaac-sim-mcp/docs/DUAL_ARM_EXPERIMENT.md).

At the user's request, [stage checkpoints](checkpoints/README.md) are saved as
timestamped USDA files before repositioning, after dynamically folding the
coupled rigs, after end placement, and after successful validation. JSON
sidecars record purpose, composition checks, and SHA256. Relative file paths
are anchored; snapshots zero velocities and hold the saved joint states. The
live source stage is not overwritten. These preserve the scene and referenced
assets, not PhysX solver state, Python controller handles, or driver processes.
The accepted resume point is
`checkpoints/20261027T071807.819020Z-03-end-table-validated.usda`, independently
reopened with all payloads loaded and no composition errors.

## Shared-object experiment and reload repair (2026-10-27)

Added `isaac-sim-mcp/scripts/shared_object_experiment.py` and the
[shared-object experiment record](isaac-sim-mcp/docs/SHARED_OBJECT_EXPERIMENT.md).
A gravity-enabled 650x35x60 mm, 0.1 kg bar is grasped at both ends through
contact/friction. USD-based FK and numerical IK generate coordinated top-down
approach and 100 mm lift paths. No object weld, kinematic motion, disabled object
gravity, or hidden support was used.

Accepted trial 004: minimum two-second hold clearance 99.388 mm, maximum slip
1.718 mm, all four fingers contacting in 100% of samples. Opening both grippers
caused a 99.522 mm gravity drop. The grasp survived checkpoint reload. The exact
final file was independently replayed with 99.495 mm minimum clearance, gravity
enabled, kinematic false, no object joints, and both finger pairs contacting.
Failed linkage-contact, initialization, and release-command trials are retained
and explained in the experiment record.

Fresh simulator playback exposed a previously missed checkpoint defect: the
old 03 stage composed correctly but second-rig joint frames had been recomputed
during posed relationship remapping. Canonical frame restoration is now in
source, and checkpoint normalization uses fully loaded payloads. Baseline 04
and final grasp checkpoint 06 passed actual reload/playback; the old 01–03
files remain historical and must not be treated as dynamically validated resume
points. Grasp snapshots preserve force-limited closing targets so preload is
not lost on reload. USD/tensor gripper targets are synchronized for release.

The current accepted checkpoint is
`checkpoints/20261027T162819.114595Z-06-shared-grasp-lift-validated.usda`.
The simulator was reopened at the user's request and is left showing the
physical shared-object hold. No physical driver was started.

## Coordinated shared-bar motion (2026-10-27)

Added `isaac-sim-mcp/scripts/coordinated_bar_motion.py`. Shared object-space
goals produce independent native arm targets through the verified USD FK/IK,
preserving both force-limited physical grasps. Tested 50 mm Y translation,
30 mm additional lift, +10 degree yaw, +5 degree pitch, return, and a final
(25,25,25) mm translated pose with -10 degree yaw. No direct object motion,
grasp weld, gain increase, or gravity change was introduced.

Accepted trial 001: 449 samples over 32.917 simulation seconds, maximum motion
slip 0.0825 mm / 0.02451 degrees, four-finger contact in 100% of samples,
minimum clearance 99.486 mm, and zero unexpected inter-rig/environment/bar
contacts under the documented mounting/fixture exclusions. Maximum object
tracking error was 1.155 mm / 0.2016 degrees. Replay retained the grasp.

Saved checkpoint 08 and replayed that exact file for two seconds: four contacts,
124.411 mm minimum clearance, no unexpected collisions, dynamic object with
gravity enabled and no object joints. Results, segment images, and replay
evidence are in `isaac-sim-mcp/resources/coordinated-bar-validation/trial-001/`.
The [motion guide](isaac-sim-mcp/docs/COORDINATED_BAR_MOTION.md) records parameters,
acceptance bounds, implementation, and limits. Prior accepted checkpoint 06
is preserved. No physical robot driver was started.

## Load/speed limits and human-readable checkpoints (2026-10-27)

Added `scripts/bar_load_speed_limits.py` and `render_bar_limit_report.py` in the
lab checkout. The motion runner now accepts command cadence, drift thresholds,
pre-load reference frames, and condition-specific checkpoint labels. A numeric
time tolerance prevents floating-point comparisons from adding extra frame
waits. The sweep reran its own baseline under this cadence.

Twenty isolated trials varied bar mass with cuboid inertia and requested path
duration, holding geometry, friction, gains, drive caps and gravity settings
fixed. Thirteen passed. Repeated mass bracket: 1.25 kg passes at 1x, 1.5 kg
exceeds the 2 mm relative-drift threshold; 2 kg drops onto the support collider
at 0.25 simulation seconds. Repeated light-bar speed bracket: 5x passes, 6x
exceeds the 10 mm path-tracking bound while grip drift stays small. This is a
controller/path boundary, not a measured physical gripper speed rating.
Combined 1.25 kg/2x and 0.625 kg/5x pass; 1.25 kg/5x fails tracking.

Successful immutable USDA originals and metadata remain in `checkpoints/`.
At the user's request, short copies now appear in `checkpoints/Approved/`:
Empty Hands, Grasp and Lift, Move and Rotate, and condition folders Heavier Bars,
Faster Moves, Combined Loads. JSON details are in Test Notes subfolders, reducing
Content-browser clutter. Every one of the eleven named load-test files was
replayed with four-finger contact, correct mass, enabled gravity, no support
contact and <0.18 mm drift. Byte/checksum integrity is retained.

Evidence and measured operating map:
`isaac-sim-mcp/resources/bar-load-speed-limits/run-001/`. The source/limits
record distinguishes actual slip, support loss, tracking lag, and untested
conditions. These are criterion-dependent simulated observations, not physical
payload ratings or generalized safety margins.

## Review and reproduction

Final rerun on 2026-10-27 passed all 12 joint cases (worst error 4.7684e-6 rad),
640x480 RGB/depth (100% valid positive depth), Vention-only support contact,
UR5e forearm contact, probe removal, surface restoration, and control recovery
(3.6061e-6 rad return error). Results are in
`isaac-sim-mcp/resources/workstation-validation/report.json` and `camera.png`.
The existing-scene workstation MCP loader also returned success. Python syntax
compilation and Git diff whitespace checks passed.

The final suite followed a rejected camera-cleanup attempt: annotator detach
needs the render product's string path rather than its HydraTexture object.
The corrected suite completed cleanup successfully. Editing live extension
Python files can reload the lab extension and restart its 8766 bridge; the
general Kit bridge on 9009 remained available, and both were restored.

Run `git status --short`, `git diff`, and `git ls-files --others --exclude-standard`
inside each checkout to review all tracked and untracked local changes. Git
diff alone omits newly added files. Keep run artifacts and failed results with
the source revisions, configuration, and simulator versions.

For the live single-arm check, import `scripts/validate_workstation.py` in the
running Isaac Sim Python context and schedule `run(output_directory)` on its
async loop through `rise-isaac.run_python`. Ordinary system Python cannot run
Omniverse APIs. Preserve the scene before testing: the check restarts physics
and leaves the arm at the raised start pose. Syntax compilation is a separate
check and does not count as simulator validation.

External configuration and downloaded runtimes are documented here, but are
not automatically versioned by these repositories. No changes have been
pushed upstream as part of this work.
