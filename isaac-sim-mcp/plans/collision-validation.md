# Single-arm collision and tracking validation

Keep the second arm absent and do not start ROS or physical drivers. Preserve existing local edits.

## Findings

- Instance-aware traversal finds seven enabled UR5e convex-decomposition colliders. Earlier ordinary USD traversal incorrectly reported none.
- The Vention CAD has 385 meshes and approximately 4.9 million vertices. Use oriented bounds per CAD part for inexpensive static collision proxies; these approximate holes and concavities.
- Gravity is disabled on the complete robot chain. Shoulder +0.03 rad stalls at 0.024876 rad, whereas -0.03 and -0.3 rad track within 4 microradians after three simulation seconds. Investigate contact before increasing gains.
- The Vention panel surface is below the legacy z=0 plane. Bench-only contact tests must disable legacy supporting surfaces temporarily and identify actual collider pairs.

## Work

- [x] Inspect loader, instance geometry, drive settings, and directional tracking.
- [x] Identify shoulder contact interference and fix the actual cause.
- [x] Persist bench collision proxy creation in the loader, keeping robot colliders intact.
- [x] Test both directions of all six joints using simulation time, RGB/depth, bench-only drop/contact, and robot contact.
- [x] Remove temporary probes, restore test settings, verify articulation control, and save measurements and limitations.

## Final evidence (2026-10-27, America/Phoenix)

`resources/workstation-validation/report.json` records `pass: true` and cleanup
completion. All 12 joint tests passed a 0.001 rad bound; the worst error was
4.7684e-6 rad, including non-commanded joints. Shoulder lift errors were
3.7849e-6 and 3.8147e-6 rad. Both color and depth are 640x480; 100% of depth
pixels were finite and positive. The bench probe settled at -0.0654746592 m,
matching the panel top plus its 0.02 m half-height; PhysX reported Vention
proxy contact. A separate probe contacted the UR5e forearm collider. Final
return error was 3.6061e-6 rad. Probes were removed, legacy surface settings
restored, and the timeline left playing. The updated workstation MCP loader
also returned success on the existing scene. Syntax compilation and diff
whitespace checks passed.

## Rejected attempts and implementation corrections

- Changing the arm's initial coordinates without moving its separately welded
  gripper caused non-finite physics transforms. Retain the original consistent
  initial state and move through drives to the raised target instead.
- A cleanup attempt passed a HydraTexture object to annotator.detach; this API
  requires its string path. Corrected and reran the complete suite successfully.
- Editing extension Python files triggers live extension reload and can restart
  its dedicated MCP bridge. The shared Kit bridge remained available on port
  9009. The lab extension/8766 bridge was restored before the final suite.

The bench proxy geometry is approximate and its actual panel remains about
85 mm below the legacy z=0 table. This suite validates contact existence and
settling, not friction, force calibration, grasp retention, or two-arm control.
