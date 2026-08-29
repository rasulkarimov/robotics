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

# How far the robot may travel on memory alone between two looks at the world.
MAX_BLIND_MM = 400

FORWARD_ANGLES = (75, 90, 105)


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
    """Median distance per bearing, in cm.

    A single echo drops out or spikes constantly on this sensor - two radar
    sweeps minutes apart disagreed by 175 cm on the same bearing - so one
    reading is never enough to authorise motion.
    """
    out = {}
    for a in angles:
        try:
            _run(["python3", "car.py", "pan", str(a)], timeout=20)
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.35)
        vals = [v for v in (_sonic_once() for _ in range(samples)) if v is not None]
        if vals:
            vals.sort()
            out[a] = vals[len(vals) // 2]
        else:
            out[a] = None
    try:
        _run(["python3", "car.py", "center-camera"], timeout=20)
    except subprocess.TimeoutExpired:
        pass
    return out


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


def preflight(action, skip_human=False):
    """action: 'drive' or 'turn'. Returns (exit_code, report dict)."""
    need = CLEAR_TURN_CM if action == "turn" else CLEAR_DRIVE_CM
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

    dist = clearance()
    report["clearance_cm"] = dist
    readable = {a: d for a, d in dist.items() if d is not None}
    if not readable:
        report["unknown"].append("no ultrasonic bearing answered")
    else:
        worst = min(readable.values())
        report["min_cm"] = worst
        if worst < need:
            report["blocks"].append(
                f"nearest obstacle {worst} cm < {need} cm required to {action}")
        if len(readable) < len(dist):
            report["unknown"].append(
                "bearings with no echo: "
                + ",".join(str(a) for a, d in dist.items() if d is None))

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
    pf.add_argument("action", choices=["drive", "turn"])
    pf.add_argument("--skip-human", action="store_true",
                    help="skip the ~60 s person check; only for a motion that "
                         "immediately follows a passed preflight")
    sub.add_parser("battery")
    cl = sub.add_parser("clearance")
    cl.add_argument("--angles", default=",".join(str(a) for a in FORWARD_ANGLES))
    hu = sub.add_parser("human", help="is a person in frame?")
    hu.add_argument("--frame")
    args = ap.parse_args()

    if args.cmd == "preflight":
        code, report = preflight(args.action, skip_human=args.skip_human)
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return code
    if args.cmd == "battery":
        v = read_battery()
        print(json.dumps({"battery_v": v, "return_at": BATT_RETURN, "stop_at": BATT_STOP}))
        return 0 if v is not None else 2
    if args.cmd == "clearance":
        angles = tuple(int(a) for a in args.angles.split(","))
        print(json.dumps(clearance(angles), indent=2))
        return 0
    if args.cmd == "human":
        human, raw = human_in_frame(args.frame)
        print(json.dumps({"human_in_frame": human, "raw": raw}, ensure_ascii=False))
        return 0 if human is False else (1 if human is True else 2)
    return 2


if __name__ == "__main__":
    sys.exit(main())
