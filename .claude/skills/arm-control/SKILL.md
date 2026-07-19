---
name: arm-control
description: Operate the Hiwonder xArm mounted on the car (home/status/move, vision-guided pick-and-place, grasp-height calibration). Use whenever a task involves moving the arm, picking up an object, or debugging arm/USB/grasp failures.
---

# Arm control

The arm is a 6-servo Hiwonder xArm bolted to the car chassis, camera mounted on the
WRIST (not the chassis - see car-control skill for the chassis-mounted ultrasonic
that confusingly uses "Camera" command names).

## Basics

- Connect via `./arm <cmd>` (repo-root wrapper) or
  `sudo /home/astra/tools/venv/bin/python3 arm.py <cmd>` directly - needs BOTH root
  (HID device access) and the venv (`hidapi`/`xarm` packages).
- `arm.py home` - go to the user-designated `HOME_POSE` (not servo-centre 500).
  `--keep-grip` if currently holding something (plain `home` opens the gripper).
- `arm.py status` - battery + all servo positions. Check this first; it also warns
  on a flat battery (`BATT_WARN`/`BATT_STOP` thresholds in arm.py).
- `arm.py step "3:400,4:820,5:470,6:500" out.jpg 900` - move one or more servos
  simultaneously and grab a snapshot in one call. This is the fast way to explore
  poses interactively.

## Joint numbering: a real discrepancy between two files

`arm.py`'s `JOINT_NAMES` calls servo 3 "shoulder" and servo 5 "wrist_pitch".
`kin.py`'s docstring says the URDF chain (verified by moving each servo alone on
camera) is actually **servo 5 = shoulder** (swings the whole arm, big effect) and
**servo 3 = wrist_pitch** (small, local tilt) - the opposite. `kin.py`'s `fk()`/`ik()`
functions use ITS OWN convention internally and don't care what you call things, but
if you're reasoning about "which servo does what" from arm.py's names alone, you
will get it backwards. When in doubt, just move one servo at a time and watch what
actually happens in a snapshot rather than trusting either file's naming.

## USB disconnects

If `arm.py` fails with `OSError: open failed` in `xarm.Controller("USB", ...)`:
1. `lsusb | grep 0483:5750` - if **absent entirely**, the arm's USB cable is
   physically unplugged; ask the user to check it (this happened mid-session once -
   confirmed by `usb 1-1.4: USB disconnect` in `dmesg`, no reconnect after). Not
   fixable from software.
2. If **present**, check `/etc/udev/rules.d/99-hiwonder-xarm.rules` covers BOTH
   `SUBSYSTEM=="hidraw"` and `SUBSYSTEM=="usb"` with `MODE="0666"` for
   idVendor=0483/idProduct=5750. The Python `hid` package here links against
   **libusb**, not the hidraw backend, so it needs write access to
   `/dev/bus/usb/BBB/DDD` (the raw USB device node), not just `/dev/hidraw0`. A
   hidraw-only rule looks right but silently isn't enough - `ls -l` the actual
   `/dev/bus/usb/...` node and compare to `lsusb`'s bus/device numbers.

## Vision-guided pick-and-place

**Canonical entry point: `tanggrab.grasp_bar(hint_base, hint_R)`** - the full proven
sequence (hint-based locate -> centre -> measure orientation -> rotated aim -> rotate
wrist -> straight descent -> clamp -> wiggle-verify -> auto-retry once). 19/20 held
across the 2026-07-19/20 drills, including reach extremes (140-195mm), base extremes
(390-660), a deliberately wrong hint (+120 units - the ring search recovered in 2
poses), and varied bar angles. Placement via `tanggrab.place()` is accurate enough
that `locate_near` re-found the bar at exactly the commanded spot 5/5 times - so
after placing, the placement coordinates ARE a reliable hint for the next grasp.

Physical constraints learned live:
- Keep reach R >= ~140mm: closer to the chassis the jaws/wrist snag on the
  chassis-mounted ultrasonic bracket (user warning after watching a near-catch).
- Pause ~1s between arm motion phases in long drills. Two spontaneous Pi reboots
  happened mid-drill during rapid back-to-back multi-servo sequences (single moves
  never triggered it, and Pi undervoltage flags stay clean on single moves);
  with 1.2s inter-phase pauses a full 5-rep drill ran clean. Suspected shared-supply
  current spikes - treat dense motion bursts as a power hazard until the supply is
  separated.

Pipeline, roughly in order of sophistication: `pick.py` (older, homography-based,
one fixed calibrated zone) -> `pick3d.py` (full 3D camera model) -> `pick_eye.py`
(current: eye-in-hand, camera+jaws are one rigid body, so the claw sits at a FIXED
pixel `rig.GRASP_PIXEL` at any height/reach - steering the object onto that pixel
IS the whole aiming problem) -> `grab2.py` (two-stage: aim wide from HIGH up, then
a timid bottom pass) -> `tanggrab.py` (rotates the wrist so jaws close across an
elongated object's SHORT axis) -> `orbit.py` (locate + multi-hop move, wraps grab2)
-> `grasp.py` (act -> verify -> retry/escalate loop wrapping tanggrab).

The object detector is a blue-blob HSV filter (`pick_eye.OBJ_LO/OBJ_HI`) - it WILL
false-positive on any other blue thing in frame (a blanket, someone's blue sleeve).
If a "found" pixel corresponds to a big blob near a frame corner rather than a
small blob near the floor, be suspicious before committing to a grasp there.

### Finding the object: use the hint first, sweep only blind

If there is ANY approximate idea where the object is (we just placed it there, the
user pointed, it was seen a moment ago) - use `orbit.locate_near(hint_base, hint_R)`:
it tries the hint pose first, then expanding rings around it. Over a 10-rep drill
where the hint came from our own last placement it found the object at the FIRST
pose every time, vs ~20 poses for a blind sweep.

Only when there's no idea at all, fall back to `orbit.locate()` (single-axis sweep
of base at one fixed R). The blob's pixel position moves smoothly and monotonically
with the base servo angle, so one coarse 1D pass is enough - don't grid-search
R x base, that's needlessly slow (learned the hard way running an 80-position grid
when a single ~20-position sweep would have found it just as well). If not found,
widen the base range before adding a second R value - a real object can sit well
outside a narrow assumed forward arc (had one at base~780 when the code's default
range was 400-580).

### GRASP_Z is not a constant - re-measure it

`rig.GRASP_Z` (the floor height in arm command-z coordinates) drifts between
sessions/mountings - the tyres, the car's lean, the arm's sag all vary. rig.py's
own docstring documents a 15mm spread as normal. If grasps keep closing on air or
grinding into the floor, re-measure live: jaws CLOSED, descend in small steps
(5-10mm), have someone watching physically confirm floor contact, then raise back
up and re-descend to the same value to confirm repeatability before trusting it.
Update `rig.GRASP_Z` (and leave a dated comment noting the old value, so the next
session can tell real drift from a fresh remount).

### Rotated (tangential-bar) grasps: skip the reach pull-in

`tanggrab.py`'s documented sequence pulls the reach IN after rotating the wrist
(`PULLIN_PER_DEG * rotation_degrees`), meant to cancel the radial swing of the
jaws. Verified 2026-07-19 (post chassis-remount): this pull-in consistently
dragged an already-good aim (6-13px error right after the rotated fine-aim) into
a bad one (100-200+px error, failed grasp) - confirmed live by the user watching
("last correction pulled you away from the target"). Skip the pull-in entirely:
after the rotated fine-aim converges, descend straight down at the SAME (x,y) to
`rig.GRASP_Z`, no radius reduction. This alone took the rotated-grasp success rate
from repeated misses to 4/4 held (one supervised + 3 unsupervised reps). The pixel
position does drift further as it descends while rotated (tracked smoothly from
~(314,138) at HIGH down to ~(372,233) at GRASP_Z in one test) - that drift is real
and not fully understood (possibly the pitch-band-constrained IK picking a
different solution at depth while rotated), but is small enough at this session's
R/heights that closing on the un-corrected aim still holds; don't try to correct
for it with a servo pass at floor height either (it reliably loses the object -
the "clipped/close-up blob" instability tanggrab.py itself warns about applies
doubly with the jaws rotated across the frame).

### The only honest success check is a wiggle test

The gripper's own servo reading LIES - a light or compressible object can read
"empty" (~618) while genuinely held, and a hard miss can read in the same range as
a good grip. Never trust servo 1's position alone. Instead: after closing and
lifting, rotate the base servo by a real amount (30-60 units, several degrees) and
check the object's pixel position via `pick_eye.see()`. A truly held object is
rigid to the camera and barely moves (a few px); an object still on the floor
slides a lot (50-200+ px) because its apparent position changes with viewing angle
(parallax). A small single-step lift-shift check can look fine even on a genuine
miss - the base-rotation wiggle is what actually catches it.

Failure statistics from a 15-rep drill (varied angles and distances, 2026-07-19):
14/15 held. The one miss came from a garbage orientation measurement (a blob
clipped at the frame edge measured long_ang=0 -> near-servo-limit wrist rotation ->
jaws closed beside the bar). The wiggle test caught it correctly, and an immediate
retry from the same position succeeded. So: on a failed wiggle test, don't
diagnose - just open the jaws and retry the whole locate->aim->grasp once from the
same hint; only escalate to a human after a second consecutive miss.
