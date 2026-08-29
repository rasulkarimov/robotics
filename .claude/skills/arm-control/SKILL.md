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
(current: eye-in-hand, camera+jaws are one rigid body, so the claw sits at a fixed
pixel for a GIVEN MOUNTING - steering the object onto that pixel IS the whole
aiming problem, but measure the pixel live rather than trusting the stored
`rig.GRASP_PIXEL`; see "The closing point goes stale" below) -> `grab2.py` (two-stage: aim wide from HIGH up, then
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


### The closing point goes stale, and it fails invisibly

`rig.GRASP_PIXEL` is only a constant for one camera mounting. Measured live on
2026-07-26 at three poses — (325,342), (392,238), (337,226) — none of them near
the stored (170,146).

What makes this the worst failure mode on the arm: the aim loop drives the object
ONTO the stored pixel, so a stale value converges *perfectly onto the wrong
point*. The logs show textbook convergence, 5-17 px, on every failed grasp.
Nothing in the output looks wrong. Four consecutive grasps failed this way. The
signature is visible only from the side: **a consistent one-sided miss** — the
user diagnosed it in one sentence ("брус попадает под левую клешню все время").

`pick_eye.measure_grasp_pixel()` does it in one move (close empty jaws, find the
two red markers, take the midpoint) and `tanggrab.grasp_bar()` now calls it
automatically. For any NEW grasp code, measure once per session. If grasps fail
while the aim looks good, suspect this before anything else.

**Take the median of three.** The first reading after a move is regularly an
outlier — (218,255) then (188,354) at one pose — because the arm is still
settling. Median-of-3 cut the spread from 100+ px to ~5 px.

**Order matters:** measure the closing point AFTER `locate_near` has put the arm
over the object, and BEFORE centring. Centring steers towards the closing point,
so measuring afterwards leaves it chasing a stale target — that drove R down to
its 120 mm clamp, past the ~140 mm floor where the jaws catch the ultrasonic
bracket, and still finished 73 px off.

### Verify a gain's SIGN by nudging, not by reading the code

`center_grabframe()` had two independent bugs that both walked it away from the
object, and the logs looked identical to "not converging". Measured live at
R=150 by nudging and watching the blob:

- base: +25 units moved the blob +122 px → **4.9 px/unit** (the code assumed 2.5,
  so every horizontal correction overshot ~2x and oscillated)
- R: −15 mm moved the blob −34.4 px → **2.29 px/mm**, and increasing R moves the
  blob DOWN the frame — the error term must be `(GY - y)`, the code had it
  inverted

Both numbers are position-specific: re-derive after any remount. A wrong sign and
a wrong gain are indistinguishable from the aim error alone.

### Aim at the height where the jaws will close, not from the hover

The closing point is fixed in the image, but the OBJECT's pixel is not — it moves
as the camera descends. Same untouched object, base=467, R=165:

| arm z | object dx vs closing point |
|---|---|
| +18 (hover) | +13 px |
| −15 | +31 px |
| −42 (grasp height) | +13.5 px, dy +56.5 px |

Aiming from 60 mm up understated the lateral error by ~18 px (~2.5 mm) and said
nothing at all about the radial error. Two closes came up empty while every cheap
signal said "good". **Align within ~10 mm of the working height, then descend
straight down** with no further lateral correction. Iterating down in steps and
re-measuring each time converges: (+13.5,+56.5) → (−11.5,−5.0) → (+5.5,+11.5).

### The stall reading cannot tell empty from held

Empty jaws read **686** on one object; a real grip on it read **676-681**. Both
sit inside the band the code calls "solid contact", so that band only rules out a
full close to clamp. It is not a grasp check. The wiggle test is (above), and for
some objects there is a better one — with the charger plug, the socket's LED
becoming visible means the plug really is in the jaws.

### Open the jaws BEFORE moving away, and open them to the right width

User, after a good insertion that came undone: "Гашение было хорошее. Ты сначала
должен разжать клещи, потом подняться." Once an object is seated or resting,
anything holding it is a mechanical link: arm motion transfers straight into the
object. So the tail of a place is always **press → measure residual → release →
only then retract**.

Release width is a second, separate decision, and the wrong one undid a good
insertion by itself: `1:156` is FULLY open, and at that width the jaw arms swing
sideways far enough to catch an object already standing in place and drag it out.
Two jobs, two numbers:

- measuring or approaching an object on the deck → **156** (kills the jaws' shadow)
- releasing something already seated → **515** (clears its width without sweeping)

And check the state right after the release step, not several moves later — on one
run a plug that had popped out was accidentally pushed back in by later arm
motion, which made a late frame look like success.

### Rotate the wrist to the object, and notice when it silently refuses

Closing across an object's SHORT axis is more accurate than closing at whatever
angle is neutral. `tanggrab.wrist_for_bar()` computes this, but it **gives up
silently and returns NEUTRAL when the measured aspect is < 1.8** — and a garbage
measurement does that on a visibly elongated object (seen live: aspect 1.32 on a
bar that is about 4:1, from a blob clipped at the frame edge). If rotation seems
not to be happening, check the measured aspect before concluding the object is
round.

### Leave the arm clean after a failure, and say which left you mean

A failed attempt that leaves the wrist rotated ~90° or the jaws half-closed makes
every frame afterwards misleading — the user caught exactly that ("Тебя ничего не
смущает? Положение клешней?"). Reset the wrist on the failure path, not at the
start of the next attempt, and check `arm.py status` before trusting a frame.

There are at least three frames in play — the robot facing forward, the camera
image, and a person looking at the robot — and switching between them silently is
a recurring, real source of error. **Say which one you mean every time** ("left in
the camera image" vs "the robot's left"), and prefer a colour/blob search over a
directional guess.

### Check it yourself before asking

Asked whether a plug was gripped, the user answered: "Ты можешь проверить сам."
The wiggle test, base rotation and several camera angles are all available.
Exhaust the robot's own senses first; escalate only after a second consecutive
genuine failure.

### The kinematics: what reaches where

`kin.py` has the real model — `fk()`, `ik()`, `ik_search()`, `reachable()`,
`max_reach()`. Constants: `BASE_HEIGHT = 67.98 mm` (base plate → shoulder axis),
`LG = 63.6 mm` (wrist axis → grasp point), `UNITS_PER_DEG = 4.0`,
`CENTER = 500` (servo 500 = arm straight up).

**The reach envelope, computed from that model:**

| height above the base plate | max reach |
|---|---|
| 0 mm (floor) | 251 mm |
| 20 mm | 246 mm |
| 40 mm | 240 mm |
| 60 mm | 230 mm |
| 80 mm | 218 mm |
| 100 mm | 201 mm |
| 120 mm | 171 mm |
| 150 mm and above | nothing |

Two consequences worth planning around. **The chassis has to park within ~25 cm
of an object on the floor**, and closer still for anything raised — driving to
"about right" and hoping the arm covers the rest does not work at 251 mm.
And the envelope collapses fast with height: the last 30 mm of lift costs 30 mm
of reach.

**`ik_search` is accurate; do not blame it for a missed move.** It searches the
approach pitch over 150-225° preferring 195° (a fixed 180° is over-constrained and
declares perfectly reachable poses unreachable). Round-tripped 542 random
solutions through `fk()` across R = 80-245 mm, z = -40..120: worst error
**0.83 mm**, which is the servo quantisation floor — 1 unit = 0.25°, about
0.87 mm of arc at R = 200. A recorded "IK returned a solution 6 mm off" from
2026-07-31 does NOT reproduce: that exact point (R=136.74, z=-37) round-trips to
0.08 mm. So when a small precise correction executes larger than intended, the
cause is mechanical — the arm did not reach the commanded position under load —
not the solver. Recompute FK from the ACTUAL servo readings after the move, which
is the check that catches it either way.

**A gotcha in the return value:** `ik_search` returns a dict keyed by servo
NUMBER (6, 5, 4, 3) plus the string key `'pitch'`. Mixed key types, so
`sorted(sol)` raises `TypeError: '<' not supported between instances of 'str' and
'int'`. Index it as `sol[5]`, `sol[4]`, `sol[3]`, `sol[6]`.

Servo resolution is the precision floor for everything above: 0.25° per unit is
~0.87 mm at 200 mm reach, so no aiming loop can do better than about a
millimetre, and asking for tenths is asking for noise.

### Do not guess joint triplets to aim the camera

Aiming the camera by trying servo-3/4/5 combinations is how the arm ends up
stretched out and drooping. Real commands from 2026-08-29, with where each one
actually puts the grasp point:

| commanded | R (reach) | z (height) | envelope at that height |
|---|---|---|---|
| `3:600,4:500,5:800` ("elbow_high") | **305 mm** | 81 mm | 217 mm |
| `3:237,4:843,5:682` (home / deck) | 106 mm | 179 mm | — |
| `3:237,4:843,5:735` (floor) | 78 mm | 201 mm | — |

The first one is 30 cm out and low: **past the reachable envelope for that
height**, and at the maximum gravity lever the shoulder will ever see. The servo
sags under it, it heats, and the current spike lands on a battery shared with the
Pi. It also puts the hand 30 cm in front of the chassis, where a drive will
swing it into furniture. That is what "the arm sat down" looks like.

Two rules:

- **To LOOK, use the named poses** — `nav.py lookout --view deck|floor|horizon`,
  or `nav.py scan --view ...` to sweep. They keep the arm folded (R = 50-106 mm)
  and only change the wrist. There is no reason to touch servos 3 and 4 to see
  something.
- **To REACH, use `kin.ik_search(x, y, z)`** and check `kin.reachable()` first.
  It returns servo values that are inside the envelope by construction. Hand-
  picked triplets are not checked by anything.

If the arm is found extended and low, `arm.py home` folds it back; do that before
anything else, because every frame taken from a drooping arm is also mis-aimed.
