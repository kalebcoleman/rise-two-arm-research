# Portable two-arm showcase

Request: demonstrate motion with Play after transfer to the school workstation,
then review and understand the project before further controller work.

Implemented October 28, 2026:

- Distinguish immutable held-pose research checkpoints from motion recordings.
- Build a separate presentation environment with embedded geometry and relative
  material files, preserving original research checkpoints.
- Add standalone Isaac launcher with Play-triggered existing controller and
  measured transform capture; export recorded USDA only after passing checks.
- Reopen exported replay and require timeline advance and sampled bar movement.
- Add transfer archive, overview, launch instructions and validation status.

Verification completed: source compilation, shell syntax, alternate-directory
USD composition without absolute home paths, object remains dynamic in start
scene, synthetic transform sampling/export with physics removed. Test artifacts
from the synthetic check stay under /tmp and are not demonstration recordings.

Completed after the owner authenticated the system repair: installed the
NVIDIA module matching running kernel 7.0.0-38, upgraded matching driver 580
components, and loaded nvidia_drm. GPU enumerates and Isaac initializes.

Fixed standalone World initialization outside the async update task, explicit
camera readback, multi-instance physics API removal, and removal of runtime
renderer graphs before export. Physics passed with four-finger contact 100%,
zero unexpected collisions, 0.135 mm max slip and 2.02 mm max tracking error.
The exported recording is split into a small readable USDA entry and a 69 MB
USDC companion. Independent Play, start/middle/end rendering, frame inspection,
and fresh-directory upstream reconstruction passed. Original checkpoints were
preserved; an Approved/00 WATCH MOTION entry points to the recording.

Research additions, raw evidence and reconstruction files are organized in
the private rise-two-arm-research repository. No controller tuning or hardware
driver work was added.
