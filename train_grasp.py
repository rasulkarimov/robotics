#!/usr/bin/env python3
"""train_grasp.py -- a self-running grasp CURRICULUM that logs every rep, so practice
produces data instead of just motion.

WHY: repeating grasp_bar() by hand tells you "it worked" or "it didn't". What actually
improves the skill is knowing WHICH conditions fail - bar angle, reach, orientation - and
the honest wiggle-shift for each. So every rep appends a row to a CSV and the summary
groups failures by condition.

The end goal this feeds is the USB plug (see memory [[usb-plug-grasp-attempt]]): the same
four capabilities, drilled here on a forgiving blue bar first.
  S1 hold      - grasp reliably at one spot (baseline; catches a stale closing point)
  S2 reach     - same, across the reach range (near/far change the pixel geometry)
  S3 orient    - bar left at varied angles: does the wrist actually rotate to match?
  S4 place     - put it down ON a target and measure the miss (this IS insertion accuracy)

Safety, learned the hard way (see memory [[single-shared-battery]]):
  - one shared pack runs Pi+arm, so a flat battery reboots the Pi mid-motion: check before
    every rep and stop at BATT_FLOOR
  - dense back-to-back multi-servo bursts triggered spontaneous reboots; PHASE_PAUSE keeps
    a gap between reps

Run under the venv as root, like everything else that touches the arm:
    sudo /home/astra/tools/venv/bin/python3 train_grasp.py --stage S1 --reps 5
"""
import argparse
import csv
import math
import os
import subprocess
import sys
import time
from datetime import datetime

sys.path.insert(0, "/home/astra/robotics")
sys.path.insert(0, "/home/astra/tools")
import kin
import pick_eye as pe
import rig
import tanggrab

LOG = "/home/astra/robotics/train_grasp_log.csv"
FIELDS = ["ts", "stage", "rep", "hint_base", "hint_R", "held", "shift1", "shift2",
          "long_ang", "aspect", "clipped", "rot", "attempts", "aim_err_px", "grasp_s",
          "place_s", "total_s", "off_centre_px", "place_err_px", "batt_v", "note"]

BATT_FLOOR = 6.6      # arm.py warns lower than this; the pack sagged to 6.47 V under load once
PHASE_PAUSE = 1.2     # seconds between reps - suspected current-spike reboots without it


def battery():
    out = subprocess.run(["sudo", "/home/astra/tools/venv/bin/python3",
                          "/home/astra/robotics/arm.py", "status"],
                         capture_output=True, text=True).stdout
    for line in out.splitlines():
        if line.startswith("battery:"):
            return float(line.split()[1])
    return None


def log_row(**kw):
    new = not os.path.exists(LOG)
    with open(LOG, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow({k: kw.get(k, "") for k in FIELDS})


def bar_state():
    """The bar's (aspect, long_ang, clipped, centre) right now, or Nones.

    Kept separate from the grasp so a rep records the CONDITION it faced even when the
    grasp itself fails - that pairing is the whole point of the log."""
    import pick
    for _ in range(5):
        m = tanggrab.measure_full(pick.frame())
        if m:
            return m[2], m[3], m[4], (m[0], m[1])
        time.sleep(0.2)
    return None, None, None, None


def one_rep(stage, rep, hint_base, hint_R, place_at=None):
    v = battery()
    if v is not None and v < BATT_FLOOR:
        print(f"BATTERY {v:.2f} V < {BATT_FLOOR} - stopping the drill")
        return None
    aspect, long_ang, clipped, centre_before = bar_state()
    t0 = time.time()
    res = tanggrab.grasp_bar(hint_base, hint_R) or {}
    grasp_s = round(time.time() - t0, 1)
    held = bool(res.get("held"))
    s1, s2 = (res.get("shifts") or (None, None))

    # Did we close on the bar's MIDDLE or near an end? While the bar is held, its centre
    # should sit at the closing point; the distance between them is how far off-centre the
    # grip is. This is the number that has to be small before the USB plug is worth trying.
    off_centre = ""
    if held:
        st = bar_state()
        if st[3]:
            off_centre = round(math.dist(st[3], rig.GRASP_PIXEL), 1)

    place_s = ""
    place_err = ""
    if held and place_at is not None:
        tp = time.time()
        pb, pr = place_at
        a = kin.s2a(pb, 6)
        tanggrab.place(pr * math.cos(a), pr * math.sin(a), rig.GRASP_Z)
        # How far off did it land? Re-find it and compare to where we aimed. This is the
        # number that matters for putting a plug INTO something.
        import orbit
        loc = orbit.locate_near(pb, pr, log=lambda *a, **k: None)
        if loc and len(loc) > 2 and loc[2]:
            place_err = round(math.dist(loc[2], rig.GRASP_PIXEL), 1)
        place_s = round(time.time() - tp, 1)

    log_row(ts=datetime.now().isoformat(timespec="seconds"), stage=stage, rep=rep,
            hint_base=hint_base, hint_R=round(hint_R, 1), held=held,
            shift1=round(s1, 1) if s1 else "", shift2=round(s2, 1) if s2 else "",
            long_ang=round(long_ang, 1) if long_ang else "",
            aspect=round(aspect, 2) if aspect else "",
            clipped=clipped, rot=res.get("rot", ""), attempts=res.get("attempts", ""),
            aim_err_px=res.get("aim_err", ""), grasp_s=grasp_s, place_s=place_s,
            total_s=round(time.time() - t0, 1), off_centre_px=off_centre,
            place_err_px=place_err, batt_v=v)
    print(f"  [{stage} rep{rep}] held={held} {grasp_s}s shifts=({s1},{s2}) "
          f"aim_err={res.get('aim_err')}px off_centre={off_centre}px ang={long_ang} "
          f"aspect={aspect} clipped={clipped} rot={res.get('rot')} batt={v}")
    return res


STAGES = {
    # (hint_base, hint_R, place_at) per rep; place_at=None leaves the bar where it lands
    "S1": lambda i: (430, 150.0, (430, 150.0)),
    "S2": lambda i: (430, [140.0, 165.0, 190.0][i % 3], (430, [140.0, 165.0, 190.0][i % 3])),
    "S3": lambda i: (430, 150.0, (430, 150.0)),   # angle varies because place() keeps rotation
    "S4": lambda i: (430, 150.0, (420, 160.0)),   # place at a DIFFERENT spot: aim accuracy
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default="S1", choices=sorted(STAGES))
    ap.add_argument("--reps", type=int, default=5)
    args = ap.parse_args()

    print(f"=== {args.stage}, {args.reps} reps -> {LOG} ===")
    ok = 0
    for i in range(args.reps):
        hb, hr, place_at = STAGES[args.stage](i)
        res = one_rep(args.stage, i + 1, hb, hr, place_at)
        if res is None:
            break
        ok += bool(res.get("held"))
        time.sleep(PHASE_PAUSE)
    print(f"=== {args.stage}: {ok}/{args.reps} held ===")


if __name__ == "__main__":
    main()
