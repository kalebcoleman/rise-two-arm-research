# Lab two-arm experiment and stage checkpoints

**Checkpoint reload correction (later on 2026-10-27):** the original 03 file
below passed live movement/contact and USD composition tests but subsequently
failed actual physics playback. It is superseded by
`20261027T161920.485684Z-04-joint-frames-repaired.usda`. Posed relationship
remapping had altered the second rig's local joint frames; canonical frames
are now restored after remapping. Checkpoint normalization now loads all
payloads so the joint types and target states are actually processed. All four
articulations passed actual reload/playback. Consult the checkpoint README for
newer accepted shared-object checkpoints; do not resume from the original 03.

Validated on 2026-10-27 (America/Phoenix), Isaac Sim 5.1.0. This is the lab
asset-based experiment, separate from the earlier generic NVIDIA two-arm demo.

## Placement and controls

| Rig | Base position, world meters | Base yaw | Arm articulation |
| --- | --- | --- | --- |
| First | `(0.75, -0.30, 0)` | 180 degrees | `/World/UR5e` |
| Second | `(-0.75, -0.30, 0)` | 0 degrees | `/World/Arm2/UR5e` |

The bases are 1.5 m apart at opposite ends of the visible table and face inward.
Each complete rig includes the supplied UR5e, quick changer, and RG2. Each arm
has six controllable arm joints; its gripper remains a separate articulation
welded to the arm. Home is `[0, -0.80, 0, 0, 0, 0]` radians. This folds the
arms above the table while allowing both to reach toward the shared center.

The second rig copies authored reference arcs/specs and remaps cross-root joint
and material targets into `/World/Arm2`. Its arm has seven enabled link
colliders, including instance-proxy geometry. No joint in the second rig targets
the first rig. Original joint gains, fixed-base anchoring, gripper coupling, and
disabled robot-chain gravity are preserved. Geometry assets are not overwritten.

Simulator articulation targets control both arms independently. The original
first-arm ROS graph remains; no second ROS graph or physical driver is launched.
These tests do not establish physical robot operation or shared-object control.

## Measured acceptance

Evidence: `../resources/dual-arm-end-table-validation/report.json`, with
`home.png`, `contact.png`, and the authored `dual_arm_scene.usda` overlay.

- All 24 cases passed: +/-0.03 rad on each of six joints on each arm, with the
  other arm held at home. The worst error across both arms was 0.000499834 rad,
  below the 0.001 rad acceptance bound, after two simulation seconds.
- No inter-rig contact at home. During the contact approach, both shoulder
  lifts moved gradually from -0.80 rad toward the center. Contact occurred at
  the -0.50 rad command, between the two **forearm collision meshes**.
- PhysX reported nonzero contact impulses and corresponding joint deflection:
  lift errors about 0.00651 / 0.00655 rad while constrained by contact. The
  approach stopped after detecting contact, then withdrew.
- Final errors were 0.00000775 / 0.00006551 rad, with no remaining inter-rig
  contact. All sampled joint states stayed finite. The scene remains playing
  at separated folded home targets.
- The two-arm overview camera is `/World/dual_arm_overview`; the existing
  workspace camera was preserved. The live viewport uses the overview.
- Python syntax compilation and Git whitespace checks passed.

The previous side layout at 0.65 m separation also passed its tests, including
upper-arm contact. Those measurements remain in `resources/dual-arm-validation/`
and are not substituted for the end-layout results above.

## Saved checkpoints

Checkpoint directory: `../../checkpoints/`. The latest accepted resume point is:

`20261027T071807.819020Z-03-end-table-validated.usda`

Earlier accepted snapshots record the pre-move stage, dynamically folded rigs,
and positioned end-layout stage before testing. Each USDA has a JSON sidecar
with purpose, UTC creation timestamp, source stage, composition check, and SHA256.
The final checkpoint was independently reopened with **all payloads loaded**;
both arms, the bench, and environment composed without errors. See
`../../checkpoints/validated-reopen-check.json` and the checkpoint README.

Snapshots preserve the coupled arm/gripper poses and joint states together.
Velocities are zeroed and driven joints hold their saved positions in the
snapshot only. This avoids restarting a prior contact command on reload. Save
new checkpoints at meaningful accepted milestones rather than replacing the
last known-good file. Source stage files are never overwritten by checkpointing.

Open a chosen USDA through the simulator's stage-open workflow after preserving
any current changes. Reacquire articulation handles after opening it. These
files reference the existing local asset tree: keep those files at their paths.
They do not bundle all assets or serialize solver caches, live Python controller
handles, ROS processes, or simulation history. Files labeled incomplete are
retained diagnostic artifacts, not accepted resume points.

## Repeating the experiment

Load the lab single-arm scene with workstation (or a saved two-arm checkpoint)
and start the simulation. Execute inside the simulator through the MCP Python
tool, using the current source directly because Kit can cache imported modules:

```python
import asyncio
from pathlib import Path
import sys
import types

name = 'rise_dual_lab'
module = types.ModuleType(name)
module.__file__ = '/home/kaleb/projects/RISE/isaac-sim-mcp/scripts/dual_arm_experiment.py'
sys.modules[name] = module
exec(compile(Path(module.__file__).read_text(), module.__file__, 'exec'), module.__dict__)
module.task = asyncio.ensure_future(module.run('/tmp/rise-dual-arm-rerun'))
```

Inspect `module.STATE` for progress and `module.task.result()` only after
`module.task.done()`. Repeated runs reuse a matching second rig; changes from
the prior layout rebuild it from complete posed specs. Stage changes occur
with physics stopped; folding and interaction use dynamic drives, not an
arm-only joint teleport that would leave the gripper weld inconsistent.

## Research limits

The mounting positions and material properties are provisional. The Vention
panel remains about 85 mm below the legacy visible z=0 support surface; bases
are fixed, not experimentally validated mounts. No friction, calibrated force,
payload, grasp retention, synchronization, collision avoidance planner, or
coordinated object manipulation has been validated. Raw solver impulses are
reported in N*s and must not be interpreted as calibrated force measurements.
