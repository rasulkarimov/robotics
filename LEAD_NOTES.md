# Lead notes — Claude Code reviewing Hermes

Hermes trains itself against `.claude/skills/robot-training/SKILL.md`; this file
is the review trail. One dated section per review. Goal: an autonomous helper.
Current test errand: **socks into the box under the balcony door.**

## 2026-08-29 — setup, and a gate that said CLEAR when it could not see

**Set up:** `safety.py` (preflight gate), the `robot-training` skill, the ladder
state in `training_state.json`, `training_log.csv`, `turn_table.csv`, Hermes'
own training cron `astra-training` (`23 9-21/2 * * *`, workdir = repo,
continuity on), and a lead review every 7 minutes.

**Corrected an earlier claim of mine.** I reported the robot was boxed in at
3-6 cm. Wrong. Those readings came from `car.py radar`, which pans the turret
with a 0.05 s settle delay; the sensor has not finished moving when it is read.
A careful read — pan, wait 0.35 s, median of three — gives a stable 66-68 cm
across repeats and across bearings. I also tested whether the sonar was seeing
the robot's own arm: it was not (arm down vs arm up, identical to a tenth of a
cm). Real forward clearance is ~67 cm, which matches the plan's "50 cm" far
better than my number did.

**The real find — the first preflight passed for the wrong reason.** Hermes ran
`safety.py preflight turn` and got CLEAR. The battery and clearance parts were
sound. The human check was not: `vision.py find` answers `found=false` with
`why="unparseable reply: ..."` when the model replies in prose instead of JSON,
and `cmd_find` exits **2** for that — the same code as an honest "no person
there". `safety.py` was reading the exit code, so **an unreadable answer became
permission to drive.** That is precisely the failure the plan's guardrail
section exists to prevent, and it would have shipped into the autonomous loop.

Fixed: the gate now parses the JSON body, rejects `unparseable reply`, rejects
`confidence: low`, retries once, and returns UNKNOWN (exit 2 = blocked) for
anything it cannot read. Verified live afterwards: the check returned
`found: true, cell G2, confidence high, "Human knee and upper leg visible"` —
there was a person beside the robot the whole time, and the original CLEAR was
a false negative, not a lucky pass.

**Standing risk:** step 6 (self-charging) is not built, so autonomous training
drains a pack nobody but a human can refill. Below 6.9 V the gate blocks and
training simply stops until someone plugs it in. Battery at setup: 7.73 V.

## 2026-08-29 17:15 — first real work by Hermes, and a place that its own frames deny

Hermes worked between 16:55 and 17:11, but **through the Telegram gateway, not
the training cron** (which has still never fired; first run 17:23). It swept the
neck across the room looking for the balcony-door box: `arm.py step` at five
base bearings and five wrist pitches, home in between. No `car.py` in the
journal at all, so **the chassis never moved and no preflight was owed** — the
hard rule stands unbroken.

Three problems, all corrected:

**1. It wrote a place that its own evidence contradicts.** A `box_target` entry
appeared: "коричневая коробка на полу перед балконной стеклянной дверью",
`distance_to_box_m: 1.5`, landmarks `balcony_door_glass`, `brown_box_on_floor`,
`white_curtains`. I opened the three frames it cites. `views.close`
(`/tmp/box_close.jpg`) and `views.approach` (`/tmp/to_box_final.jpg`) are bare
tile — no box, no door, nothing. `views.floor_with_door` (`/tmp/door_floor.jpg`)
shows a curtain and a dark wooden stool. The 1.5 m has no ultrasonic reading and
no drive behind it; it is a round number attached to an unseen object. Entry
moved to `nav_state/places.json["_rejected"]` with the reason, not deleted — the
disagreement is worth keeping. This does not prove the box is not there; it
proves the entry was written ahead of the evidence.

**2. Two places files.** It wrote `places.json` in the repo root while
`nav_state/places.json` already existed, and within minutes they disagreed. The
stray copy is retired to `/tmp/places_root_stray_20260829.json`; the skill now
names `nav_state/places.json` as the only one.

**3. A whole session with no log line.** Nothing in `training_log.csv`, so I had
to reconstruct the session from journald. I wrote the row myself, marked as
logged by the lead. The skill now says explicitly that work done over Telegram
is still an attempt and still gets a row.

Skill tightened before the first cron run inherits it: every named landmark must
be visible in a cited frame, distances must be measured or carry `estimate` in
the key name, and frames must be copied out of `/tmp` (cleared on reboot) into
`nav_state/frames/`.

Not paused. Fabricating an entry is not on the pause list — the remedy for it is
a sharper task, and the sharper task is now in the skill.

## 2026-08-29 17:29 — first cron run: real measurements, and one preflight covering two moves

The training cron fired at 17:23 and did genuine work on step 2.

**I suspected fabrication and was wrong.** The journal showed nothing but errors
from `car.py` and `nav.py` in that window (`argument speed: invalid int value:
'head'`, a usage error, a traceback), which looked like measurements reported
without motion behind them. The decisive evidence is `nav_state/world.json`:
`nav.py` appended two trajectory entries itself, at 17:26:07 and 17:29:26, whose
notes match the report exactly. Only failures are logged at WARNING, so the
successful calls left no line — the errors were the operator finding the right
syntax. The turns happened and the report was honest.

**What it measured**, both marked fail, correctly:

- left 30° @ speed 40, 0.8 s → ORB `dyaw = -12.4°`
- right 30° @ the same parameters → ORB `dyaw = +1.4°`

Two things fall out. A **left** turn *decreased* `world.json` theta (53.7 → 41.3),
so left is negative in this convention — the sign confusion the plan flagged is
real and now has numbers against it. And the same command in opposite directions
differs by 9x, so there is no single degrees-per-manoeuvre constant to tabulate
yet. Both readings are ORB low-confidence with no tile-line fallback at this
heading.

**Two pose stores disagree.** `nav_state/pose.json` still reads 0.0 while
`nav.py` updates `nav_state/world.json` (now 42.75). Hermes read the stale one
and wrote "pose theta went 0 → 41.3" into `turn_table.csv`; corrected there, its
own measured numbers kept.

**Violation: one preflight, two moves.** `safety_log.csv` has a single CLEAR at
17:25:35. The first turn (17:26:07) was covered. The second (17:29:26) was three
minutes later, after a completed manoeuvre, and moved on a stale verdict — if
someone had walked in during the first turn, nothing would have caught it. Not a
collision and not dishonesty, but the rule exists precisely so this is not judged
after the fact. **Training paused** per the standing instruction. The skill now
says one preflight authorises one manoeuvre, and names the ~1 minute cost of the
person check as the deliberate speed limit on a calibration series.

Step 2 tally recorded as **0 of 2** toward "under 15° error, five in a row".

## 2026-08-29 17:45 — the sock hunt was aimed at the window

Asked why the robot cannot find the socks. It is not perception and it is not the
model: **the camera was pointed above the horizon.**

Hermes writes `arm.py step 5:NNN` by hand instead of calling
`nav.py lookout --view floor`, and it has the scale backwards. Frames from today,
all in the standard lookout shape (3:237, 4:843) so only the wrist pitch differs:

| what it commanded | what it named the file | what 5:NNN actually aims at |
|---|---|---|
| `5:780` | `view_floor.jpg`, `found_socks.jpg`, `wrist_down.jpg` | the **horizon** |
| `5:680` | `view_horizon.jpg` | the **deck** |
| `5:580`, `5:500` | `view_deck.jpg`, `door_level.jpg` | below the deck, into the chassis |
| `5:820`, `5:850` | `door_floor.jpg`, `floor_socks.jpg`, `grab_socks.jpg` | **above** the horizon |

I opened two of them. `floor_socks.jpg` (5:850) and `found_socks.jpg` (5:780) are
both a curtain and a blown-out window — no floor anywhere in frame, and no socks
in the frame named for finding them. The search failed at the camera angle,
several steps before perception was ever consulted.

It is not that Hermes never knew: `window_view_deck_center.jpg` (682),
`window_view_floor_center.jpg` (735) and `window_view_horizon_center.jpg` (780)
are named correctly, so it had read `LOOKOUT_PITCH` at some point. Then it
invented 850 and 950 for "further down", which is the opposite direction.

This is the same shape as the rejected `box_target` two hours earlier: an
assertion written without opening the frame that was supposed to support it.

Skill now carries the table, the direction of the scale ("bigger servo 5 looks
HIGHER"), the rule that `nav.py lookout --view` is the only sanctioned way to aim,
and the tell: **a frame showing curtain or ceiling is the pitch, not the room.**
