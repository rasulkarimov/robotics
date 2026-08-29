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
