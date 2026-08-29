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

Append one row per attempt to `training_log.csv` — including the attempts that
found nothing, and including work done outside a cron run (a scout over Telegram
is still an attempt). On 2026-08-29 a full scouting session left no row at all
and the lead had to reconstruct it from the journal.


    ts, step, what_was_tried, measured, verdict(pass/fail/blocked), note

Then update `training_state.json`. A step is only "passed" when its criterion is
met by the tally in that file — not by a good feeling about the last run.

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
