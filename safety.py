#!/usr/bin/env python3
"""Preflight gate the autonomous operator must pass BEFORE any chassis motion.

The training plan says the limits have to live in code rather than in good
intentions, and the reason is concrete: on 2026-08-29 a human hand was in frame
while the robot was driving, and the robot was parked a few centimetres from
furniture while its own notes claimed 50 cm of room.

Runs under the SYSTEM python3 (like car.py and vision.py), not the arm venv, so
it can be called from anywhere. The only arm-venv call is `./arm battery`, which
the wrapper handles itself.

Exit codes:
    0  clear to move
    1  BLOCKED - a limit says no (reason on stdout as JSON)
    2  UNKNOWN - a sensor could not be read; treat exactly like BLOCKED

Never treat "the check crashed" as permission to drive: 2 is not 0.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(REPO, "safety_log.csv")

# Battery. The pack is shared by the Pi, the motors and the arm
# (see AGENTS.md), so these govern the whole robot, not just the arm.
# RETURN sits ABOVE arm.py's BATT_WARN on purpose: an errand that only aborts at
# the warning threshold has no charge left to reach the charger with.
BATT_RETURN = 6.9   # abort the errand, drive to the charger
BATT_STOP = 6.8     # stop moving at all

# Clearance. The chassis is ~200 mm long and a K-turn swings the tail, so
# "enough room to turn" is much more than "enough room to creep forward".
CLEAR_DRIVE_CM = 25.0
CLEAR_TURN_CM = 45.0

# A third regime, added 2026-09-09 at the user's insistence and they were right:
# "Ты сейчас не катаешься а работаешь с предметом, тебе нужно подкорректировать
# свое положение."
#
# The two thresholds above answer "is there room to TRAVEL". They are wrong for
# positioning against something you are deliberately working on, where the close
# object IS the target: the robot was 16 cm from an open bag it was trying to
# drop a bar into, and a 25 cm rule made the task impossible by construction.
#
# So a nudge is its own action, and it is narrow on purpose: a few centimetres,
# under supervision, with the bumper still refusing anything inside genuine
# collision range. It buys a working distance, not permission to drive.
CLEAR_NUDGE_CM = 8.0
NUDGE_MAX_CM = 10.0

# How far the robot may travel on memory alone between two looks at the world.
MAX_BLIND_MM = 400

FORWARD_ANGLES = (75, 90, 105)

# Two echoes from genuinely different bearings are never bit-identical; when they
# are, the sensor is stuck rather than measuring. Real repeat readings on the same
# bearing vary by whole centimetres, so 0.05 cm is far below the noise floor and
# cannot fire on a true measurement.
STUCK_EPS_CM = 0.05

# THE SONAR IS A BUMPER, NOT A RANGEFINDER.
# User, 2026-09-09: "Не ориентируйся на сонар. Если только для избежания
# столкновения, дальше 30 см он не работает."
#
# This invalidates how this gate worked all evening. CLEAR_TURN_CM is 45, and a
# sensor that is meaningless past 30 cm can never honestly satisfy it - so every
# turn verdict derived from a sonar number above 30 was noise, whether it blocked
# or allowed. Beyond this range a reading is NOT distance and must not be read as
# "there is room"; it is simply no information.
#
# So: depth is the primary clearance sense (validated 0.15-4 m, with coverage as
# its confidence), and the sonar may only ever ADD a block when it reports
# something genuinely close. It can no longer clear anything.
SONAR_TRUST_CM = 30.0

# Depth (Aurora930 RGB-D, served by aurora-camera.service on :8090/depth) is the
# PRIMARY clearance sense, as of 2026-09-09 - see SONAR_TRUST_CM for why it had to
# take over. It is a 640x400 metric field, validated from ~0.15 m to ~4 m, and
# `min_cm` in the report is now its number.
#
# Its own blind spots, both measured, and both why the sonar bumper stays:
#   * nothing inside ~15 cm, which is exactly where a collision happens;
#   * a NARROW VERTICAL object is under-weighted, because the sector figure is a
#     percentile over a wide band and most of that band sees past a thin pole.
DEPTH_MIN_COVERAGE = 0.18
# Measured 2026-09-09, five samples each and stable to a few tenths of a percent:
# arm folded/home (the pose a drive is supposed to start from, looking at the
# near floor) gives 26%; arm at `horizon` pitch, staring at a plain wall past the
# backlit counter, gives 11%. 0.18 sits between them, so a gate that demands
# coverage is also demanding that the arm be looking where the robot would drive.


def _run(cmd, timeout=60):
    return subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=timeout)


def read_battery():
    """Volts, or None if the arm did not answer."""
    try:
        p = _run(["./arm", "battery"], timeout=40)
    except subprocess.TimeoutExpired:
        return None
    for tok in p.stdout.replace("battery:", " ").split():
        try:
            v = float(tok)
        except ValueError:
            continue
        if 3.0 < v < 12.0:
            return v
    return None


def _sonic_once(timeout=20):
    try:
        p = _run(["python3", "car.py", "ultrasonic"], timeout=timeout)
    except subprocess.TimeoutExpired:
        return None
    for line in reversed(p.stdout.strip().splitlines()):
        try:
            d = float(line.strip())
        except ValueError:
            continue
        # 0.0 is the sensor's dropout value, not a real 0 cm reading.
        return d if d > 0 else None
    return None


def clearance(angles=FORWARD_ANGLES, samples=3):
    """(median distance per bearing in cm, every raw reading taken).

    A single echo drops out or spikes constantly on this sensor - two radar
    sweeps minutes apart disagreed by 175 cm on the same bearing - so one
    reading is never enough to authorise motion.

    The raw readings come back too because a frozen sensor is invisible in the
    medians: see is_sonic_stuck().
    """
    out, raws = {}, []
    for a in angles:
        try:
            _run(["python3", "car.py", "pan", str(a)], timeout=20)
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.35)
        vals = [v for v in (_sonic_once() for _ in range(samples)) if v is not None]
        raws += vals
        if vals:
            vals.sort()
            out[a] = vals[len(vals) // 2]
        else:
            out[a] = None
    try:
        _run(["python3", "car.py", "center-camera"], timeout=20)
    except subprocess.TimeoutExpired:
        pass
    return out, raws


def is_sonic_stuck(raws, eps_cm=STUCK_EPS_CM, need=3):
    """True when every echo came back byte-identical across different bearings.

    This sensor's dead mode is not silence, it is a plausible constant. On
    2026-08-29 it returned 24.837 cm nine times running; on 2026-09-09 it
    returned 173.502 cm on fifteen reads spanning 140 deg of pan, while the depth
    camera saw a sofa at 67 cm - and the raw I2C register was frozen too, so it
    is the shield/sensor, not Main.py (a car-server restart does not clear it).

    The gate cannot survive that on medians alone: a constant answers every
    bearing, so `readable` is full and `min_cm` looks generous. The turret really
    does move between bearings, so identical values across them are not a
    measurement - they are the absence of one.
    """
    return len(raws) >= need and (max(raws) - min(raws)) < eps_cm


def depth_clearance():
    """(nearest_cm | None, coverage 0-1 | None, note).

    Nearest is the 5th percentile of non-zero depths in the central band, not the
    single minimum - one stray near pixel is noise, and the gate should not be
    hostage to it. None means the depth map could not be read at all, which is an
    unknown, not a clear.
    """
    try:
        if REPO not in sys.path:
            sys.path.insert(0, REPO)
        import depth as depth_mod
        w, h, a = depth_mod.fetch()
        s = depth_mod.sectors(w, h, a)
    except Exception as e:
        return None, None, f"unreadable: {type(e).__name__}: {e}"[:160]
    # USE THE SECTOR YOU ARE DRIVING INTO, NOT THE WHOLE BAND.
    # sectors()["nearp"] is the 5th percentile of left+centre+right POOLED, so far
    # surfaces off to the sides dilute a near obstacle dead ahead. Measured
    # 2026-09-11, standing in front of the lamp: centre 70 cm, pooled 96 cm - the
    # gate authorised a 1.1 m drive on a number 26 cm more optimistic than the
    # thing it was about to hit. depth.py's own `clear` CLI already preferred
    # `center`; this is the gate catching up with it.
    near = s.get("center")
    note = "ok (centre sector)"
    if near is None:
        near = s.get("nearp")
        note = "ok (no centre data; pooled band)"
    return (None if near is None else near / 10.0), s.get("coverage"), note


def human_in_frame(frame=None, retries=1):
    """(bool_or_None, raw_answer). None means the question could not be answered,
    which counts as unsafe - not as 'nobody there'.

    Do NOT read vision.py's exit code here. `find` returns found=false with
    why="unparseable reply: ..." when the model answers with prose instead of
    JSON, and cmd_find exits 2 for that exactly as it does for a genuine miss.
    On 2026-08-29 that turned an unreadable answer into a CLEAR verdict and a
    green light to drive. The JSON body is what distinguishes the two.
    """
    if frame is None:
        frame = "/tmp/safety_human_check.jpg"
        try:
            p = _run(["python3", "car.py", "snapshot", frame], timeout=40)
            if p.returncode != 0:
                return None, "snapshot failed"
        except subprocess.TimeoutExpired:
            return None, "snapshot timed out"

    last = "no attempt"
    for _ in range(retries + 1):
        try:
            p = _run(["python3", "vision.py", "find", frame,
                      "a person, or any part of a person such as a hand, arm, "
                      "foot or leg"],
                     timeout=200)
        except subprocess.TimeoutExpired:
            last = "vision timed out"
            continue

        obj = None
        for line in reversed(p.stdout.strip().splitlines()):
            line = line.strip()
            if line.startswith("{"):
                try:
                    obj = json.loads(line)
                except ValueError:
                    obj = None
                if obj is not None:
                    break
        if obj is None:
            last = "vision printed no JSON: " + (p.stderr or p.stdout).strip()[-200:]
            continue

        why = str(obj.get("why", ""))
        if why.startswith("unparseable reply"):
            last = "vision could not be parsed: " + why[:200]
            continue
        if obj.get("found"):
            return True, json.dumps(obj, ensure_ascii=False)[:300]
        # A confident "no person" is the only answer that authorises motion.
        if str(obj.get("confidence", "low")) == "low":
            last = "vision unsure: " + json.dumps(obj, ensure_ascii=False)[:200]
            continue
        return False, json.dumps(obj, ensure_ascii=False)[:300]

    return None, last


def log_row(action, verdict, detail):
    new = not os.path.exists(LOG)
    with open(LOG, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["ts", "action", "verdict", "detail"])
        w.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"), action, verdict,
                    json.dumps(detail, ensure_ascii=False)])


def preflight(action, skip_human=False, direction="forward", rear_cm=None):
    """action: 'drive' or 'turn'. Returns (exit_code, report dict).

    `direction` matters because every sensor on this robot faces FORWARD. The
    ultrasonic bumper is mounted at the front, so it says nothing whatever about
    reverse - and on 2026-09-09 a version of this gate that ignored direction
    blocked the robot from backing AWAY from the glass door it was 24 cm from,
    while also (correctly) blocking it from going forward. A gate that leaves no
    legal move is not a safety device, it is a trap.

    For a reverse the caller must supply `rear_cm` - a rear clearance it has
    actually measured, e.g. by aiming the arm camera at the rear quarters and
    reading depth.py. There is no sensor that does this on its own.
    """
    need = {"turn": CLEAR_TURN_CM, "nudge": CLEAR_NUDGE_CM}.get(action, CLEAR_DRIVE_CM)
    report = {"action": action, "need_cm": need, "blocks": [], "unknown": []}

    v = read_battery()
    report["battery_v"] = v
    if v is None:
        report["unknown"].append("battery unreadable")
    elif v < BATT_STOP:
        report["blocks"].append(f"battery {v} V below stop threshold {BATT_STOP}")
    elif v < BATT_RETURN:
        report["blocks"].append(
            f"battery {v} V below return threshold {BATT_RETURN} - charger only")

    dist, raws = clearance()
    report["clearance_cm"] = dist
    readable = {a: d for a, d in dist.items() if d is not None}
    stuck = is_sonic_stuck(raws)
    report["sonic_stuck"] = stuck
    if stuck:
        report["unknown"].append(
            f"ultrasonic frozen: {len(raws)} readings across "
            f"{len(readable)} bearings all equal {raws[0]} cm - not a measurement")

    # Sonar as a bumper only: a reading inside SONAR_TRUST_CM is a real object
    # and blocks; anything beyond it carries no information and is ignored.
    close = {a: d for a, d in readable.items() if d < SONAR_TRUST_CM}
    report["sonar_close_cm"] = close or None
    report["direction"] = direction

    # THE BUMPER CAN BE ABSENT, AND THAT MUST NOT BE SILENT.
    # The user physically removed the ultrasonic on 2026-09-11. Every bearing then
    # returns 0.0, _sonic_once maps that to None, `readable` is empty - and this
    # gate happily returned CLEAR with no mention of it. The old "no ultrasonic
    # bearing answered" unknown was lost when the sonar was demoted to a bumper
    # and the dead branch around it was deleted.
    #
    # Demoting it was right; losing the notice was not. Depth is the primary
    # sense, but it is blind inside ~15 cm and returns nothing off glass - which
    # is precisely the pair of cases the bumper existed to cover. Driving with
    # neither is a real gap, so say so, and refuse unless depth is BOTH
    # well-covered and reporting generous room.
    report["sonar_present"] = bool(readable)

    if close and not stuck and direction == "forward":
        nearest = min(close.values())
        # SONAR_TRUST_CM decides which readings are BELIEVABLE; `need` decides
        # which are too close to move. Conflating them made the bumper refuse a
        # nudge at 11 cm against an 8 cm floor - it was applying the trust range
        # as if it were the limit, so no action could ever be closer than 30 cm
        # to anything, which defeats the whole point of a manipulation nudge.
        if nearest < need:
            report["blocks"].append(
                f"ultrasonic bumper: something at {nearest} cm < {need} cm "
                f"required to {action}")
        else:
            report["note_bumper_cm"] = nearest
    elif close and direction == "backward":
        report["note_forward_obstacle_cm"] = min(close.values())

    if direction == "backward":
        report["rear_cm"] = rear_cm
        if rear_cm is None:
            report["unknown"].append(
                "reversing with no rear clearance given - nothing on this robot "
                "faces backwards; measure it and pass --rear-cm")
        elif rear_cm < need:
            report["blocks"].append(
                f"rear clearance {rear_cm:.0f} cm < {need} cm required to {action}")
    d_cm, d_cov, d_note = depth_clearance()
    report["depth_cm"] = d_cm
    report["depth_coverage"] = d_cov
    if d_cm is None:
        # Only an unknown for a FORWARD move: that is the direction depth is the
        # primary sense for. Reversing is judged on rear_cm, and forward blindness
        # says nothing about it. Glass is the case that makes this bite - the
        # Aurora gets zero returns off the balcony door, measured 0% coverage at
        # 24 cm, so a direction-blind rule would strand the robot against it.
        if direction == "forward":
            report["unknown"].append(
                f"depth map {d_note} - and it is now the PRIMARY clearance sense, "
                "since the sonar is only trusted as a close-range bumper")
        else:
            report["note_depth_forward"] = f"no forward depth ({d_note}); "\
                                           "irrelevant to a reverse"
    else:
        report["min_cm"] = d_cm          # the number the verdict actually rests on
        if d_cm < need and direction == "forward":
            report["blocks"].append(
                f"depth sees a surface at {d_cm:.0f} cm < {need} cm required to {action}")
        # Low coverage means the depth map has no opinion. That is only safe to
        # shrug off when the ultrasonic independently reports plenty of room -
        # otherwise "no returns" is exactly what an obstacle inside the camera's
        # ~15 cm blind zone looks like.
        # Forward-only, for the same reason as the unreadable case above: this is
        # a statement about the sense that judges FORWARD travel. Applying it to a
        # reverse is the second instance of the same bug - a front-facing sensor's
        # failure forbidding the one direction that leads away from the obstacle.
        if direction == "forward" and d_cov is not None and d_cov < DEPTH_MIN_COVERAGE:
            us = report.get("min_cm")
            if us is None or us < 2 * need:
                report["unknown"].append(
                    f"depth coverage {d_cov * 100:.0f}% below "
                    f"{DEPTH_MIN_COVERAGE * 100:.0f}% (is the arm folded and looking "
                    "at the near floor?) and the ultrasonic does not independently "
                    "show generous room")

    # Resolve the absent-bumper rule now that the depth figures exist.
    if not report.get("sonar_present"):
        cov = report.get("depth_coverage")
        dm = report.get("depth_cm")
        strong = (cov is not None and cov >= DEPTH_MIN_COVERAGE
                  and dm is not None and dm >= 2 * need)
        if strong:
            report["note_no_bumper"] = (
                f"ultrasonic absent; proceeding on depth alone ({dm:.0f} cm at "
                f"{cov*100:.0f}% coverage). Blind inside ~15 cm and to glass.")
        else:
            report["unknown"].append(
                "ultrasonic absent (no bearing answered) and depth is not "
                f"independently strong (cm={dm}, coverage={cov}). Nothing is "
                "watching the close range or glass.")

    if skip_human:
        report["human"] = "skipped"
    else:
        human, raw = human_in_frame()
        report["human"] = human
        report["human_raw"] = raw
        if human is True:
            report["blocks"].append("a person is in frame")
        elif human is None:
            report["unknown"].append("could not tell whether a person is in frame")

    if report["blocks"]:
        code, verdict = 1, "BLOCKED"
    elif report["unknown"]:
        code, verdict = 2, "UNKNOWN"
    else:
        code, verdict = 0, "CLEAR"
    report["verdict"] = verdict
    log_row(action, verdict, report)
    return code, report


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    pf = sub.add_parser("preflight", help="full gate; run before every motion")
    pf.add_argument("action", choices=["drive", "turn", "nudge"],
                    help="nudge = a few cm of positioning against an object you are "
                         "working on, not travel; see CLEAR_NUDGE_CM")
    pf.add_argument("--direction", choices=["forward", "backward"], default="forward",
                    help="which way the move goes; every sensor here faces forward")
    pf.add_argument("--rear-cm", type=float, default=None,
                    help="rear clearance in cm, measured by the caller - required to reverse")
    pf.add_argument("--skip-human", action="store_true",
                    help="skip the ~60 s person check; only for a motion that "
                         "immediately follows a passed preflight")
    sub.add_parser("battery")
    cl = sub.add_parser("clearance")
    cl.add_argument("--angles", default=",".join(str(a) for a in FORWARD_ANGLES))
    hu = sub.add_parser("human", help="is a person in frame?")
    hu.add_argument("--frame")
    sub.add_parser("depth", help="nearest surface + coverage from the depth map")
    args = ap.parse_args()

    if args.cmd == "preflight":
        code, report = preflight(args.action, skip_human=args.skip_human,
                                 direction=args.direction, rear_cm=args.rear_cm)
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return code
    if args.cmd == "battery":
        v = read_battery()
        print(json.dumps({"battery_v": v, "return_at": BATT_RETURN, "stop_at": BATT_STOP}))
        return 0 if v is not None else 2
    if args.cmd == "clearance":
        angles = tuple(int(a) for a in args.angles.split(","))
        dist, raws = clearance(angles)
        stuck = is_sonic_stuck(raws)
        print(json.dumps({"clearance_cm": dist, "raw": raws, "sonic_stuck": stuck},
                         indent=2))
        return 2 if stuck else 0
    if args.cmd == "human":
        human, raw = human_in_frame(args.frame)
        print(json.dumps({"human_in_frame": human, "raw": raw}, ensure_ascii=False))
        return 0 if human is False else (1 if human is True else 2)
    if args.cmd == "depth":
        d_cm, d_cov, note = depth_clearance()
        print(json.dumps({"nearest_cm": d_cm, "coverage": d_cov,
                          "min_coverage": DEPTH_MIN_COVERAGE, "note": note}))
        return 0 if d_cm is not None else 2
    return 2


if __name__ == "__main__":
    sys.exit(main())
