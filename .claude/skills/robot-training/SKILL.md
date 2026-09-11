---
name: robot-training
description: The training ladder Astra works through on its own - one checkable step at a time, with the hard limits that must stop it. Use at the start of every autonomous training run, before deciding what to practise, and whenever a step's criterion is being judged met or failed.
---

# Training ladder

You are the operator. Nobody is watching each run. Work ONE step at a time, prove
it with a criterion, write the result down, and stop when a limit says stop.

The ladder exists because the first errand ("fetch socks, put them in the box")
failed and nobody could say which of its four stages broke.

**The current test errand is: socks into the box under the BALCONY DOOR.** That
box is the target for step 5, and it is the place worth recording first under
step 3 - not the window. The point of the whole exercise is an assistant that
finishes an errand on its own. Each step below is a
stage isolated so a failure names itself.

## The one rule that outranks everything

**No chassis motion without a passed preflight.**

    python3 safety.py preflight drive     # before driving
    python3 safety.py preflight turn      # before a K-turn (needs more room)

Exit 0 = go. Exit 1 = BLOCKED. **Exit 2 = UNKNOWN, which means a sensor did not
answer — treat it exactly like BLOCKED.** A check that crashed is not permission.

**One preflight authorises ONE manoeuvre.** Not a series, not a calibration
session. On 2026-08-29 a run preflighted once at 17:25 and then turned twice,
three minutes apart — the second turn moved on a verdict about a world that had
already changed underneath it. If a person walked in during the first turn,
nothing would have noticed. Re-run it before every single move, and always after
a completed drive or more than ~2 minutes.

The person check costs about a minute, which is the real reason to be tempted.
That cost is the point: it is what makes an unattended robot safe to leave
moving, and it caps how fast a calibration series can run.

**Fold the arm before you drive.** An extended arm sits in the ultrasonic's beam
and the gate then measures the robot's own gripper instead of the room. On
2026-08-29 this read a steady 24-29 cm and blocked the drive; folding the arm to
the lookout shape changed the same reading to 83 cm with nothing else moved. A
held object makes it worse, because the sock is exactly what the beam hits.
Suspect it whenever clearance is oddly constant and oddly close.

**The gate's person check needs the wrist camera, which a held object blocks.**
Carrying something therefore disables the one check that protects a person. Until
there is a second camera, an errand that carries an object is a supervised
errand: a run with `--skip-human` does not count toward any criterion, and the
log line has to say a human was watching.

`safety.py` enforces, in code: battery return/stop thresholds, forward clearance,
and "is a person in frame". It writes every verdict to `safety_log.csv`.

## Current position on the ladder

`training_state.json` holds `current_step` and the tally so far. Read it first,
write it last. Do not skip ahead: a step whose criterion is unmet is where the
next failure will come from.

| # | Step | Criterion |
|---|------|-----------|
| 0 | Wake and report | diagnostics + short report. **Already passing.** |
| 1 | Find an object in a frame, say where | 8 of 10 correct, and **zero** inventions on frames that have no object |
| 2 | Turn a commanded angle | error under 15 deg, five attempts in a row |
| 3 | Drive to a named place | 5 of 5 arrivals within 30 cm |
| 4 | Pick an object off the floor | 4 of 5 held, judged by the wiggle test |
| 5 | Socks into the box **under the balcony door**, end to end | 3 full runs out of 5, no human hands. **Inside the box, verified** — on the rim does not count |
| 6 | Charge yourself | 5 of 5 docks from anywhere in the room |

Step 2 is the real blocker and the hardest: one K-turn measured **2.4 deg**, and
after five left turns the pose claimed +25 deg CCW while the view matched a
NEGATIVE bearing — the heading sign or scale is wrong somewhere between `turn`,
`NECK_SIGN` and the bearing math. Do not paper over this with an averaged fudge
factor. Find the sign first, then build the table.

## How to practise step 2 (heading calibration)

1. Preflight `turn`. If BLOCKED for clearance, say so and stop — this step needs
   real floor space and the room may simply not have it today.
2. Put a fixed reference in view (the tile seams work; they are documented in
   `AGENTS.md`) and snapshot before.
3. Command ONE known manoeuvre. One. Not five.
4. Snapshot after and measure the actual rotation two ways: the tile seams, and
   `vision.py` on the before/after pair.
5. Record steer, speed, duration, commanded, measured, and the SIGN into
   `turn_table.csv`.
6. Only once one manoeuvre is understood, vary one parameter at a time.

If the chassis physically cannot exceed ~10 deg per manoeuvre, that is a finding,
not a failure: record it, and plan routes as long arcs instead of spot turns.

## Looking for something: never hand-pick the wrist pitch

Use `nav.py lookout --view <deck|floor|horizon>` or `nav.py scan --view <...>`.
Do not write `arm.py step 5:NNN` yourself. The three values are measured against
real frames and the named view is the whole point of them existing.

**The scale runs the way you would not guess: a BIGGER servo 5 looks HIGHER.**

| servo 5 | view | what is in frame |
|---|---|---|
| 682 `deck` | steeply down | about a metre of bare tile, plus the jaws |
| 735 `floor` | down and out | the floor from ~1 to 3 m — **this is the one for an object on the ground** |
| 780 `horizon` | level | furniture, doorways, the far wall |
| above 780 | up | curtains, window, wall. Nothing on the floor is here. |

On 2026-08-29 a sock hunt ran at **780 and 850** with the frames named
`view_floor.jpg`, `floor_socks.jpg`, `found_socks.jpg`. Every one of them shows a
curtain and a blown-out window — no floor in frame at all, and `found_socks.jpg`
contains no socks. The search failed at the camera angle, before perception got a
chance. 780 is ALREADY the horizon; there is nothing above it but wall.

There is also no "look further down" below `deck`: 682 is already steep, and 500
or 580 just points into the robot's own chassis.

So: object on the floor → `--view floor`. Furniture, a doorway, a box against a
wall → `--view horizon`. Something on the robot's own deck → `--view deck`. And
if a frame comes back showing curtain or ceiling, that is the pitch, not the room.

**And do not reach for servos 3 and 4 either.** Guessing triplets to aim the
camera is what leaves the arm stretched out and sagging: `3:600,4:500,5:800` puts
the hand 305 mm out at 81 mm high, past what the arm can hold at that height, at
the worst possible gravity lever — on a battery shared with the Pi. The named
views keep it folded at 50-106 mm and move only the wrist. See the arm-control
skill, "Do not guess joint triplets to aim the camera".

## Releasing into a box: over the edge is NOT enough

The plan says a box needs no precise seating, just open the jaws over the edge.
That is half right and it cost a run on 2026-08-29: released at R=190 over the
box's near rim, the sock landed ON the rim, half in and half out.

A rim is not an interior. Aim the release point **past** the near wall — 20-30 mm
further out, or drive 5-10 cm closer — so that what falls, falls inside.

And verify before releasing, not after. From 19 cm the wrist camera looks OVER a
low box: the box was not in frame at the moment the jaws opened. If the target
cannot be seen together with the jaws, back off 20 cm first and look — that is
what finally showed where the sock had landed.

### 2026-09-11: the procedure that actually put the bar inside

Superseding the guesswork above. Four steps, each producing a number you can log:

1. **Approach on bearing, not on distance.** Re-measure the target's pixel column
   after every leg and correct. Bearing needs only `fx`/`cx`; it survived two
   jammed wheels mid-run. See car-control.
2. **Swing the base until the target sits under the held object's column.** The
   held bar is fixed in the image, so this is a pixel comparison, not a 3D
   estimate: bag centroid was 18.3 deg left of the bar; 20.5 deg of base rotation
   closed it. (Rotation near the target also translates the camera, so the effect
   is smaller than the command — 9.25 deg of base gave 6 deg of closure. Iterate.)
3. **`place_until_contact` decides rim vs hole.** 112 mm of free descent, then
   contact at z=-81.3 against a floor of -75. See arm-control.
4. **Release to 515 (not wide), verify the grip moved (635 → 526), lift
   VERTICALLY, then photograph the object where it lies.**

Note on `goto_vertical`: lifting straight up out of a container gets refused
partway (it pulls R inward toward `R_MIN_CHASSIS` — the documented failure). Lift
in absolute `goto` steps that grow R as z rises: (170,10) → (180,40) → (175,60).

### The 20 cm rim verdict was about the bag's SHAPE, not the task

`rig.BAG_RIM_TOO_TALL_MM = 200` says clearing a 20 cm rim leaves a 5 mm corridor
between `R_MIN_CHASSIS` and the edge of the envelope, and calls the task
unsolvable. That stands as arithmetic. But the same bag, **slumped so the mouth is
wide and low**, was solved end to end on 2026-09-11 — the arm never had to clear a
rim at all, it descended straight through an opening wider than the gripper.

So read that constant as "an upright 20 cm rim is out of reach", not "this bag is
out of reach". Before believing it, check the bag's present shape. And do not
measure that shape from a shallow-pitch frame: my "the rim is 145 mm / 202 mm /
270 mm" estimates that evening were all the bag's extent ALONG THE GROUND read as
height, because at 36-47 deg of pitch image-vertical is mostly ground-forward.

## The fetch chain, with the numbers that worked

Run end to end on 2026-08-29 (sock → box at the balcony door). Follow it in this
order; every number here was measured, not assumed.

1. **Look, with a named view.** `nav.py scan --view floor` for the room. For
   something within ~30 cm, the lookout shape with a lower pitch:
   `3:237,4:843,5:500,6:<bearing>`. Sweep the BASE, never servos 3/4.
2. **Find the bearing by sweeping, not by reasoning.** Increasing servo 6 looks
   LEFT. An empty frame from a 35 deg camera is not evidence of absence — move
   and look again before concluding anything.
3. **Descend from the pose that has the object in view.** Do NOT compute a hover
   over "where the hand should be": the camera looks along the gripper axis and
   sees about 79 mm BEYOND the grasp point, so that hover lands short and shows
   bare floor. This wasted most of an hour.
4. **Grasp.** Open (`arm.py move 1 156`), descend to `rig.GRASP_Z` = -75, close.
   On the sock: closing to 660 stalled at 642 and it fell later; closing to 700
   stalled at **679** and held. Squeeze past first contact on a soft object.
5. **Verify by wiggling**, then **do not fold it home** — `arm.py home` dropped
   the first grasp. Lift within the same pose family.
6. **Fold the arm before driving.** An extended arm reads as an obstacle to the
   sonar (24-29 cm of nothing).
7. **Turn with arcs, not K-turns.** `car.py step forward 60 0.6 --steer right
   --angle 45` ≈ 20 deg. A K-turn gave 0.1 deg the same day.
8. **Drive gated.** Speed 55: 0.5 s ≈ 5 cm, 1.4 s ≈ 27 cm. Read clearance before
   every step and stop at the threshold.
9. **Get the target in frame WITH the jaws before releasing.** From 19 cm the
   wrist camera looks over a low box. If you cannot see both, back off 20 cm and
   look — before opening the jaws, not after.
10. **Release past the near wall**, not over it, and to ~380-515 rather than
    fully open.

## Places, not coordinates

Dead reckoning does not survive here — a return after 10 K-turns landed somewhere
else entirely. Record places and arrive by matching the current view to the
stored one. That routes around the missing odometry rather than fighting it.

**The file is `nav_state/places.json`, and only that one.** A second copy was
written to the repo root on 2026-08-29 and the two immediately disagreed; the
next run would have read whichever one it happened to open.

**A place is only recorded if its own frames prove it.** Before you write an
entry:

- Every landmark you name must be VISIBLE in a frame you cite. Do not name
  `brown_box_on_floor` in an entry whose frames show bare tile.
- Any distance must come from a reading (`safety.py clearance`, ultrasonic) or
  be written as `distance_estimate_m` with the word estimate in the key. A round
  number with nothing behind it is the thing the reviewer looks for first.
- Copy the frames out of `/tmp` into `nav_state/frames/`. `/tmp` is cleared on
  reboot, and an entry whose evidence has evaporated cannot be checked.
- **`/tmp` HAS ALREADY DESTROYED EVERY PLACE RECORD ONCE.** Checked 2026-09-09:
  places.json cited 42 frames, all under `/tmp`, and **0 of 42 still existed** -
  two reboots had wiped them. Seven places (start_position, window_spot,
  search_spot_1, charging_spot, box_spot, and both turn calibrations) are now
  unverifiable and are marked `evidence_status` in the file rather than deleted.
  Write frames straight into `nav_state/frames/`; never cite a `/tmp` path.
- Record the RANGE per bearing too, now that depth exists: sweep servo 6 at the
  named pitches and store `depth.py` nearest + coverage per bearing. That is what
  makes a place re-findable by matching, instead of by dead reckoning that does
  not survive here. `sofa_side_2026_09_09` is the worked example.
- If you looked and did not find the thing, that is a perfectly good result:
  write it in `training_log.csv` and record no place. An entry that claims more
  than the frames show is worse than no entry, because the next run trusts it.

## Limits that are not negotiable

- **A person in frame stops everything.** Stop, wait, re-shoot, re-preflight.
  A hand appeared in frame during motion on 2026-08-29; this is not theoretical.
- **Do not grasp anything off the whitelist.** This is a workshop: soldering
  iron, power strip, cables, other people's chargers are REPORTED, never picked
  up. Unsure means do not touch.
- **Never drive blind.** At most 400 mm between two looks at the world.
- **Battery reserve.** Below 6.9 V abort the errand and head for the charger;
  below 6.8 V stop moving. The return threshold is deliberately above the alarm
  so there is charge left to reach the charger with. One pack feeds the Pi too,
  so a sag reboots the whole robot mid-motion.
- **Geofence: this room.** The hallway and kitchen are visible but off limits
  without explicit permission.
- **Every autonomous action gets a log line.** Autonomy without a post-mortem
  does not improve.

## Writing results down

**Sound-wake rows are written by `listen.py` itself, not by you** — it holds the
clock and the measurements. Four runs in a row invented distances there, the last
of them after an explicit ban, and one stamped a row eleven minutes in the
future. That is not dishonesty, it is writing a report from an impression into a
column that asks for measurements; the fix is to take the pen away rather than
repeat the instruction. For everything you DO log:

`training_log.csv` has SIX columns in this order and exactly one header line,
at the top, written once:

    ts,step,what_was_tried,measured,verdict,note

`step` is a LADDER NUMBER, 0-6, never a word: a sound wake is step 0, a grasp is
step 4. `measured` holds only quantities something actually measured. A distance
you judged by eye is still not a measurement, and three runs in a row wrote
"20-40 см", "~1 м", "~60 см" as though they were observations - one of them after
an explicit ban. That discipline stands.

**What changed 2026-09-09: the wrist camera now HAS depth.** This rule used to
say "do not write a distance at all", because the sonar faces forward and cannot
see the bearing the arm is pointed at, and the old webcam was monocular. The
Aurora930 is an RGB-D camera and `depth.py` reads a 640x400 metric map, so the
robot can finally measure the thing it is looking at.

So a distance MAY be written when, and only when, it is quoted with its source:

- `depth.py ranges` / `depth.sectors()` - give the number in mm **and its
  coverage**. Coverage IS the confidence: measured live, a bearing at 3% coverage
  read "1.28 m" and meant nothing, while the same sweep's 78% bearing read 0.39 m
  and was the sofa. **Below ~15% coverage, write "no depth", not a number.**
  Two riders added 2026-09-11, both bought with a wasted hour:
  **(a)** low coverage on a specific OBJECT usually means it is too CLOSE to
  measure, not too dark — slice its pixels by image row and look at which bands
  are silent before you interpret it;
  **(b)** for an object, quote the **5th percentile**, never the median: the
  median of a large object describes its far side, and mine said 541 mm about a
  bag whose near rim was at 207 mm.
- `car.py ultrasonic` - quote the reading. And check it is not frozen first
  (`safety.py clearance` reports `sonic_stuck`); on 2026-09-09 it returned
  173.502 cm on fifteen reads across 140 deg of pan.

Depth returns nothing closer than ~15 cm or past ~4 m, so an object at the
robot's own base still has no measurable distance - that is a "no depth", not a
guess. Otherwise describe WHAT you see and WHERE in the frame.

And never quote **a frame's minimum depth as the sensor's minimum range**. It is
just the nearest object in view; across one evening mine read 699, then 490, then
388, then 207 mm. I took the first as a specification and built a plan on it.

Never append a second header, and never reorder the columns. On 2026-08-30 a run
wrote its own header mid-file plus rows in two different layouts; the file stopped
parsing and the lead had to repair it by hand. It is the shared evidence base -
a log that cannot be read proves nothing.

Append one row per attempt to `training_log.csv` — including the attempts that
found nothing, and including work done outside a cron run (a scout over Telegram
is still an attempt). On 2026-08-29 a full scouting session left no row at all
and the lead had to reconstruct it from the journal.


    ts, step, what_was_tried, measured, verdict(pass/fail/blocked), note

Then update `training_state.json`. A step is only "passed" when its criterion is
met by the tally in that file — not by a good feeling about the last run.

## The recurring failure: naming a measurement, then never testing the name

This is the same mistake four times now, and it is worth more attention than any
single technique in this file.

| what I named it | what it was | what betrayed it |
|---|---|---|
| "distance is non-linear in duration" (2026-09-09) | two wheels mechanically jammed | the user, not me |
| "the drive is stuck" / "direction is inverted" (2026-09-09) | the gate had REFUSED the move; nothing had run | the exit code I did not read |
| "low depth coverage = dark fabric" (2026-09-11) | 85% of the object was too close to measure | coverage per image row |
| "the dark V is the bag's mouth" (2026-09-11) | the threshold behind the bag | range·sin(pitch) > camera height |

The shape is always the same: a reading arrives, I attach an interpretation, and
then I reason for an hour from the interpretation while never once testing it
against something independent. The data to catch it was in hand every time.

**The habit that fixes it.** Before building on any named quantity, spend one
line on a check that could falsify the NAME, not refine the number:

- a geometric bound — `range·sin(pitch) ≤ camera_height`, or the point is not on
  the floor in front of you;
- a second sensor — sonar 46 cm against depth 48 cm agreed; sonar 26 cm against a
  claimed 54 cm did not, and I ignored it;
- a pattern rather than a scalar — coverage BY ROW, not coverage;
- "did this actually happen?" — the exit code, the servo readback, the grip value.

And when the check cannot be made: say the name is provisional. Today's honest
version — "the camera pitch does not solve, so I will not range by floor plane,
I will range by raw depth on the object" — was right, and led straight to the
useful measurement.

## When you are stuck

Stop and write it down; do not improvise around a hard limit or invent a number
to make a criterion pass. A blocked run that is honestly logged is more useful to
the lead reviewer than a run that "worked" for reasons nobody can reconstruct.
Say plainly which of the four stages broke.

## What already works - do not rebuild it

- `vision.py` — general object finding (`find` needs ~3500 tokens, ~60 s per
  call; a 5-bearing sweep costs 5 minutes of motor and battery). See the
  `vision` skill.
- `nav.py lookout --view horizon|floor|deck` — 780 for furniture and doorways,
  735 for objects on the floor, 682 for the deck.
- `pick_eye.py` — the Jacobian servo loop. Vision gives the first aim point, this
  finishes the approach. Never go straight from a vision cell into a grasp.
- `usb-charging` skill — the plug transfer, with its measured poses.
- Grasp lore lives in the `arm-control` skill: measure the closing point live,
  wiggle to verify, and open the jaws BEFORE withdrawing.
