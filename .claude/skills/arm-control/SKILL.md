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

### Finding the object: one coarse sweep is enough

The blob's pixel position moves smoothly and monotonically with the base (neck)
servo angle. A single-axis sweep at one fixed R (e.g. `orbit.locate()`) is enough -
don't grid-search R x base, that's needlessly slow (learned the hard way running an
80-position grid when a single ~20-position sweep would have found it just as
well). If not found, widen the base range before adding a second R value - a real
object can sit well outside a narrow assumed forward arc (had one at base~780 when
the code's default range was 400-580).

### GRASP_Z is not a constant - re-measure it

`rig.GRASP_Z` (the floor height in arm command-z coordinates) drifts between
sessions/mountings - the tyres, the car's lean, the arm's sag all vary. rig.py's
own docstring documents a 15mm spread as normal. If grasps keep closing on air or
grinding into the floor, re-measure live: jaws CLOSED, descend in small steps
(5-10mm), have someone watching physically confirm floor contact, then raise back
up and re-descend to the same value to confirm repeatability before trusting it.
Update `rig.GRASP_Z` (and leave a dated comment noting the old value, so the next
session can tell real drift from a fresh remount).

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
