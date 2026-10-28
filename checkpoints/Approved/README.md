# Human-readable approved checkpoints

**Open `00 WATCH MOTION.usda` and press Play to see the recorded motion.**
The other files restore held poses. Keep the adjacent `isaac-sim-mcp/showcase`
folder with its companion files. See [the demo guide](../../isaac-sim-mcp/docs/SHOWCASE.md)
for recorded playback versus a fresh physics run.

Open this folder in Isaac Sim Content:

`/home/kaleb/projects/RISE/checkpoints/Approved/`

| File | What opens | Verified |
| --- | --- | --- |
| `00 WATCH MOTION.usda` | 26-second recorded bar translation and rotation | Fresh physics, exported Play and rendering verified |
| `01 Empty Hands.usda` | Two opposite-end arms, folded, no shared bar | Reload/playback finite after joint-frame repair |
| `02 Grasp and Lift.usda` | Both grippers holding the bar above the table | Physical grasp, lift, gravity release and exact-file replay |
| `03 Move and Rotate.usda` | Bar held after coordinated translation and rotation | Slip/contact/collision bounds and exact-file replay |

These are byte-identical copies of accepted timestamped files in the parent
folder. The originals and JSON provenance remain there. File 03 is the newest
accepted light-bar manipulation milestone. Load tests below are named by mass and speed.
Earlier rejected/pending snapshots are deliberately excluded from this folder.

The parent filenames are:

- 01: `20261027T161920.485684Z-04-joint-frames-repaired.usda`
- 02: `20261027T162819.114595Z-06-shared-grasp-lift-validated.usda`
- 03: `20261027T164045.283144Z-08-coordinated-bar-motion-validated.usda`

The right-hand Content pane's "Location does not support Checkpoints" refers
to the browser's version-history feature. These USDA files are ordinary saved
stages: open the named file itself rather than that version-history pane.

## Heavier/faster tests

Open these folders for short condition names rather than timestamps:

| Folder / file | What passed |
| --- | --- |
| `Heavier Bars/1.25kg - 1x.usda` | Highest passing baseline-speed mass |
| `Faster Moves/0.1kg - 5x.usda` | Highest passing light-bar speed scale |
| `Combined Loads/1.25kg - 2x.usda` | Heavier bar with faster movement |
| `Combined Loads/0.625kg - 5x.usda` | Moderate mass at the highest passing speed scale |

Other passing cases are kept alongside these. All eleven load-test browser
files passed physical replay. The `Test Notes` subfolders contain JSON details;
only USDA stages are shown alongside those folders. Opening a stage restores
the held pose and mass; it does not automatically play a motion recording.

At 1x, 1.5 kg exceeded the grasp-drift threshold and 2 kg dropped. At 0.1 kg,
6x exceeded the path-tracking bound even though grip slip remained small.
See [measured limits](../../isaac-sim-mcp/docs/BAR_LOAD_SPEED_LIMITS.md) for the
criteria, boundaries, observed speeds, failures and untested conditions.
