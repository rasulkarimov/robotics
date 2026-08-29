---
name: usb-charging
description: Move the USB plug from the robot's own chassis socket into the wall charger's socket (the self-charging task). Use for any pick/insert of the charger plug, and before trusting any stored pose, pixel or sensitivity for it.
---

# Self-charging: chassis plug -> charger socket

The plug lives in a socket **on the robot's own chassis**. The job is to take it from
there and seat it in the charger's socket. Everything below was learned over ~12 live
runs (2026-07-28 .. 08-01), most of which failed the first time for a reason recorded here.

## Read this before any number below

**The two ends are different problems. Do not build one mechanism for both.**

- **Pick (chassis socket): a taught pose, valid forever.** The socket rides on the
  chassis with the arm base, so its servo values do not change when the car drives.
  Verified: after `drive_mm(backward,100)` + `drive_mm(forward,100)`, going straight to
  the stored pose gripped on the first try with no re-aim.
- **Place (charger socket): needs vision every single time.** The car parks imprecisely,
  so the charger moves in the arm's frame. It has also physically shifted mid-session
  (56 px between runs) while the plug was being handled.

**Absolute numbers here are a starting hint for a search, never a pose to execute.**
What is durable is *relations between two things visible in the same frame* (the GO
picture, the glow trend). What is not durable: every base/R/z value, every closing
point, every px-per-mm sensitivity. They belong to one parking spot and one pose family.
Re-derive them live. This is not caution for its own sake — two runs failed precisely
because a stored value was reused.

Battery: the pack powering the arm is the one being charged
(see `AGENTS.md`, "Питание"). Attempt this well before `BATT_STOP`, not as a last resort —
a sagging battery makes the manoeuvre that fixes it less reliable.

## 1. Pick from the chassis

Working numbers (`base=311, R=76, z=40`), stall **679-681** on a real grip.
No hover stage needed; going straight to z=40 works.

```
grab2.pose(311, 76, 40) -> close servo 1 to 820 -> check stall in 679-681
```

- `z=40` is user-supplied and was corrected upward twice. z=20 is **too low** ("Низко
  взял"). The plug's base sits ~100 mm above the floor — the same height as the arm's
  own deck, which is one reference point, not two.
- `R=76` is far inside the arm-control skill's "keep R>=140" rule. That rule is about
  floor objects and the ultrasonic bracket; it cannot apply to the chassis' own socket.
  Check bracket clearance, don't treat the close reach as the fault.
- Do lateral centring at z~45+ and only then descend — aiming at deck level scrapes the
  cardboard.

## 2. Transit (holding the plug)

**Never `arm.py home` while holding anything — it opens the gripper.** Transit by
setting servos 3/4/5 to the home *shape* (`237/843/682`), leaving servo 1 alone, and
rotating the base only in that raised shape. Rotating low over the deck risks the
chassis/ultrasonic bracket.

**Restore the pitch before descending.** After transiting in the home shape the pitch is
~106°, i.e. the plug is nearly horizontal; it must be brought back to ~194.5° before it
can enter the socket.

**The grip offset shifts the insertion depth.** The chassis plug is gripped ~5 mm above
its base; a charger-end regrip measured 21 mm. That 16 mm difference moves where the jaws
must stop. Compute it from where you actually gripped — never reuse the old insertion z.

## 3. Place into the charger — measuring

**`pick_eye.measure_grasp_pixel()` returns `None` on this background.** Its red mask is
hardcoded `S>=60` (`pick_eye.py:275-276`) and the tan cardboard reads `S~114`, so both jaw
pads merge with the whole panel into one ~87k px blob, over `MARKER_AREA_MAX`. **Raise
S to ~140** (pads measure `S~172`); at 160 they vanish. Sample a few HSV points from a
fresh frame rather than assuming 140 still fits the light.

- With jaws closed the two pads nearly touch — morphological CLOSE merges them. Use
  **OPEN** and take the two bounding-box centres.
- `measure_grasp_pixel(samples=3)` already medians 3 readings; the first reading after a
  move is regularly an outlier by tens of px.
- **Measure the closing point at the working pose and aim immediately.** Three samples at
  one pose agree to <1 px, but *re-approaching the same pose* lands ~7 px away. Every
  extra move in between re-rolls that 7 px — and ~7 px is exactly the "немного в бок" the
  user has complained about. This is the dominant error term, not the detector.

**Finding the plug against cardboard: use a row profile, not a blob.** The plug merges
with the jaw assembly into one ~51k px component that then fails the area cap. Scan
`V<40..60` along rows ~150-300, take the longest dark run per row, median the row centres
over ~11 clean rows (single rows scatter 187-210 px on one frame). Reject runs that are
implausibly wide — a run widening to 80-115 px below the body is **the cable**, and
including it drags the centroid off. Verify the centre is stable across V<50/60/70; if it
moves, the mask has caught shadow.

**Open the jaws wide (`1:156`) before measuring.** The open jaws throw a shadow onto the
plug and a dark mask locks onto shadow+plug together. The tell is an absurdly *low*
probed sensitivity (1.0 px/mm when the truth was 5.1) because the merged blob's centroid
barely moves. Widening the jaws collapsed a 42 px error to 0 in one step.

## 4. Aim

**Align at the height where the jaws will actually close**, within ~10 mm — not from a
hover. The closing point is a rigid-body constant, but the object's pixel is not: on one
untouched plug, dx read +13 px at hover, +31 px at mid-descent, +13.5 px (dy +56.5) at
grasp height. Aiming from 60 mm up understated the lateral error by ~18 px and said
nothing about reach. Iterate down in steps, re-measuring each time.

**Probe sensitivities at CONSTANT pitch or they are garbage.** With pitch pinned:
~6.0-6.5 px/mm radially (object moves UP in frame as R *decreases*) and ~4.5-6 px per
base unit laterally. Letting pitch vary gave 1.6 px/mm *and the opposite sign*.

**Base backlash is 5-8 units here, not the 3 in `rig.BASE_BACKLASH_UNITS`.** Commanding
311->316 moved nothing; motion resumed at 322. At 4-8 px/unit that is 30-60 px of dead
zone, so a small correction after a direction change is a no-op — chasing dx below ~15 px
with another base command is chasing noise. **Always approach the final base angle from
the same direction** (overshoot past it, then come back), so the lash is taken up
consistently. That took dx from +180 px to -5 px in two moves.

**Never pixel-match against frames from a different pose.** Camera tilt shifts the whole
image ~14 px/deg (~35° FOV over 480 px), so comparing centroids across a 5° pitch change
invents a 60-110 px phantom error and sends corrections the wrong way.

## 5. The GO/NO-GO picture — check before committing to the descent

This is the **durable** criterion, because it is a relation between two things in one
frame. It holds at any parking spot and any pose:

- `|socket_cx - plug_cx|` within a few px, and
- the socket's lower edge sits immediately above the plug's upper edge, no visible gap.

If the centres disagree, fix it with base rotation **before** going down. Correcting
laterally during the descent is what produces the forward drift that catches the socket's
front edge. Tolerance is ~2-3 px; a USB shell has less clearance than the 7 px once
accepted as "close enough". Do not rationalise a residual because it looks small.

**Start the descent at R ~139.** Forward drift vs starting reach is monotonic, not noise:

| start R (mm) | 135.2 | 136.4 | 138.9 | 139.2 |
|---|---|---|---|---|
| forward drift (mm) | +5.66 | +4.58 | +0.65 | +1.00 |

## 6. Descend — the glow is the reach check

At pitch ~185 the camera looks nearly straight down, so the image gives a trustworthy
**lateral** axis and nothing usable for reach (probing R by 6 mm moved the socket/plug gap
by 1 px). Servo lag does not fill the gap either — it only says the plug hit *something*,
and a 4 mm z-shortfall was recorded on a good seat *and* on a run that ended with the plug
lying beside the socket.

**Count glow pixels at each descent step.** Monotonic decay toward ~0 = going in. A
minimum followed by a **rise** = sliding off to the side, abort and re-aim:

| z | -20 | -30 | -36 | -40 | -43 |
|---|---|---|---|---|---|
| failed | 10156 | 5649 | 5693 | 6262 ↑ | 7108 ↑ |
| good | 5341 | 3137 | 729 | 531 | 440 |

The divergence is clear by z=-36, several steps **before** contact. This does not
contradict the rule that the glow is useless for *alignment* — its centroid/area rides
auto-exposure (area doubled 3306->7826 px between two near-identical frames). What is
robust is the occlusion **trend**, a change far larger than the ~2% exposure noise.

**Use the glow for the socket's cx while the socket is still empty.** Twice, the
dark-opening detector locked onto the *held plug's own top* instead of the socket and
reported a stubborn dx of -12..-14 px that would not respond to base commands. **That
non-response is the tell**: a clamped plug cannot move in frame when the base rotates. If
"the socket" doesn't move, you are measuring the plug.

Descend in ~4-5 mm steps, reading servo lag each step: 2-4 units = free air, 6+ = contact.

## 7. Seat, then release — order matters

**Press down, not forward.** The push itself is wanted ("ты его должен вниз жать, а не
вперед"). Under contact a multi-servo move does not complete proportionally — some joints
reach the angle, others stall, and the tip leaves the straight-down line. A press
commanded as pure -z came out as +0.69 mm forward; a deeper one landed 7.7 mm off.
**Commanded geometry is not achieved geometry once anything touches.**

1. press in 1-3 mm steps, watching servo lag (free-air slack is 3-8 units)
2. recompute FK from the **ACTUAL** `arm.py status` readings and measure the residual (dR, dz)
3. **open the jaws first — to `1:515`**
4. only then null the forward drift, retract, rise

**Release before withdrawing** (user: "Ты сначала должен разжать клещи, потом
подняться"). Once seated, the socket holds the plug; any arm motion with the jaws still
closed transfers straight to the plug and pulls it back out. This cost a run.

**Release to 515, not 156.** `GRIPPER_OPEN = 515` in `usb_charger.py` "clears the block's
width, measured live". 156 is *fully* open and the jaw arms swing far enough sideways to
catch a plug already standing in the socket, dragging it out on the way up. Two different
jobs, two numbers: **measuring on the deck -> 156** (kills the shadow); **releasing a
seated plug -> 515**. This cost another run.

When only verifying a grip (wiggle test), realign the base to the original angle **before**
opening — otherwise the plug is set back down off-centre.

Also: `kin.ik_search` returned a solution whose FK was ~6 mm off the request (asked
R=136.74, got 130.9), turning an intended 4.1 mm retraction into 9.8 mm. **FK-verify any
small precise correction before executing it.**

## 8. Confirming the result

- **Right after the release step, not several moves later.** On one run the plug fell out
  and was then accidentally pushed back in by later arm moves — a good-looking final frame
  is not proof.
- **Glow area, compared only against a same-pose reading.** Occupied read 731 px and
  1178 px after retreat, against ~5000-6000 px empty at that pose. Absolute figures
  (6942/8063/0) belong to one fixed pose; a 15756 px reading was once misread as "empty,
  insertion failed" when the plug was in fact seated.
- **The battery reading cannot confirm charging.** `arm.py status` reports whatever powers
  the robot right now. There is no self-check for charging success — it needs a human.

## Code that already exists

- `usb_charger.py` — `POSE_GRASP/POSE_EXTRACT/POSE_SEAT` are hand-taught **charger-socket**
  poses, position-specific and only valid with the car parked exactly where they were
  taught. They are not the chassis source. Read its docstring before reusing anything.
- `grab2.pose(base, R, z)` — the pose helper used for the chassis pick.
- `pick_eye.py` — `measure_grasp_pixel()` (see the S>=60 problem above),
  `goto_vertical()`, and `OBJ_LO/OBJ_HI` (blue) which is *not* a safe detector for this
  object: the plug is small, dark and compact, and its own LED is blue.
- `rig.py` — `GRASP_Z=-75`, `ARM_ABOVE_FLOOR=110`, `BASE_BACKLASH_UNITS=3` (too small here).

**There is still no automated detector for this plug.** Until one exists, steer it the way
the user did live: snapshot -> threshold-locate -> compare against the live closing point
-> small nudge -> fresh frame, and let a human correction override any read of blurry
footage. See the arm-control skill for the arm.py/kin.py joint-naming discrepancy before
reasoning about which servo does what.
