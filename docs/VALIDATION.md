# Verified demo baseline — October 28, 2026

The accepted recording begins with a 0.1 kg bar already held by two RG2 grippers.
Its 26.07-second sequence translates, yaws, pitches, returns and moves to a final
pose. It records measured rigid-body transforms from a passing simulation.
Playback disables physics; the live launcher performs a fresh physics test.

| Check | Result |
| --- | --- |
| Fresh presentation-scene physics | Passed |
| Four-finger contact | 100% of measured samples |
| Unexpected collisions | 0 |
| Maximum translation / rotational slip | 0.135 mm / 0.036 degrees |
| Minimum clearance | 99.5 mm |
| Maximum position / rotation tracking error | 2.02 mm / 0.354 degrees |
| Independent exported USDA Play | Passed; bar moved 16.93 mm in first 2 s |
| Start, middle, end rendering | Passed and visually inspected |
| Pinned upstream reconstruction with local clone cache | Passed |
| Downloaded WATCH alias composition | Passed; all USD layers resolve inside fresh checkout |

Evidence: `isaac-sim-mcp/showcase/recordings/report.json`,
`replay-manifest.json`, `playback-verification.json`, and replay/segment PNGs.
The playback report contains SHA256 hashes for the USDA entry and its USDC
companion. Earlier research evidence and rejected trials remain under
`isaac-sim-mcp/resources/` and `checkpoints/`; those bytes were preserved.

The GPU failure was a missing module for the running kernel, repaired with
owner-authenticated Ubuntu packages. See the GPU repair record. Standalone
launcher fixes addressed nested async initialization, explicit camera capture,
multi-instance physics-schema removal and stale renderer graphs in exports.

Source compilation and shell syntax passed. Generated USD, upstream license
text and patch files retain their original formatting; they are not rewritten
to satisfy whitespace checks on a newly imported evidence collection.

No controller tuning or real-hardware operation was added. Contact and actuator
parameters remain uncalibrated, and the imported model is not yet a validated
physical digital twin. See `START_HERE.md` for the staged research roadmap.
