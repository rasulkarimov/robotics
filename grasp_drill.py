#!/usr/bin/env python3
"""Repeat a pick-and-place until the numbers say it is reliable.

Written by the lead after two operator-authored drill scripts died the same way
before touching the arm: one folded to HOME_POSE and opened the jaws there,
dropping the object from 18 cm; the other recovered joint angles by regex from
`arm status`, a message written for people, and raised on the first call - which
sat right after the grasp, so the arm would have been left holding the object in
mid-air with the script dead. Both would have been caught by running the
functions once. Writing this here removes the need to invent it again.

**Parameterised on purpose.** The grasp pipeline underneath is object-agnostic -
`center_grabframe` works on blob pixels and measured gains - so a different
object needs a different DETECTOR, not a different drill. Point `--base/--R` at
it, and if its colour differs, that is `pick_eye.OBJ_LO/OBJ_HI`.

    grasp_drill.py --dry-run          # print the plan, move nothing
    grasp_drill.py --reps 5
    grasp_drill.py --reps 5 --base 470 --R 154

Runs under the ARM VENV (needs cv2/numpy) and as root:
    sudo /home/astra/tools/venv/bin/python3 grasp_drill.py --reps 5
"""
import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

REPO = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, REPO)
LOG = os.path.join(REPO, "training_log.csv")

# The bar as measured 2026-08-30: hard against the base, in front. `deck` (682)
# and `floor` (735) both look past it - see the arm-control skill.
DEF_BASE, DEF_R, DEF_PITCH = 470, 154, 500
# 515 clears the object without sweeping it; 156 is fully open and the jaw arms
# swing wide enough to knock over what was just set down.
RELEASE_WIDTH = 515
LOOKOUT = "3:237,4:843"


def log_row(what, measured, verdict, note):
    with open(LOG, "a", newline="") as f:
        csv.writer(f).writerow([datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
                                "4", what, measured, verdict, note])


def place_down(arm, kin, rig, base, R, log=print):
    """Put the held object down: descend, THEN open, THEN lift.

    The order is the whole point. Opening at height is a drop, and moving away
    while still closed drags the object with it.
    """
    ang = math.radians((base - 500) / 4.0)
    for z in (rig.GRASP_Z + 60, rig.GRASP_Z + 20, rig.GRASP_Z):
        sol = kin.ik_search(R * math.cos(ang), R * math.sin(ang), z)
        if not sol:
            log(f"  no IK at z={z:.0f}; stopping the descent here")
            break
        arm.setPosition([[6, sol[6]], [5, sol[5]], [4, sol[4]], [3, sol[3]]],
                        duration=900, wait=True)
        time.sleep(0.4)
    arm.setPosition(1, RELEASE_WIDTH, duration=700, wait=True)   # release FIRST
    time.sleep(0.5)
    sol = kin.ik_search(R * math.cos(ang), R * math.sin(ang), rig.GRASP_Z + 70)
    if sol:                                                      # then lift
        arm.setPosition([[5, sol[5]], [4, sol[4]], [3, sol[3]]],
                        duration=900, wait=True)
    time.sleep(0.3)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--base", type=int, default=DEF_BASE)
    ap.add_argument("--R", type=float, default=DEF_R)
    ap.add_argument("--pitch", type=int, default=DEF_PITCH)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    print(f"drill: {a.reps} reps at base {a.base}, R {a.R:.0f}, "
          f"search pitch {a.pitch}, release {RELEASE_WIDTH}")
    if a.dry_run:
        print("dry run: nothing moved")
        return 0

    import kin, rig, tanggrab
    from xarm import Controller
    arm = Controller("USB")

    held = 0
    for i in range(1, a.reps + 1):
        print(f"\n--- rep {i}/{a.reps}")
        try:
            res = tanggrab.grasp_bar(a.base, a.R)
        except Exception as e:
            print(f"  grasp_bar raised: {e}")
            log_row("grasp drill rep", f"base {a.base}, R {a.R:.0f}",
                    "fail", f"grasp_bar raised: {str(e)[:100]}")
            continue

        if not res or not res.get("held"):
            print("  not held")
            log_row("grasp drill rep",
                    f"base {a.base}, R {a.R:.0f}, result {res}",
                    "fail", "wiggle says not held (grasp_bar retried once already)")
            continue

        held += 1
        print(f"  HELD: {res}")
        place_down(arm, kin, rig, res.get("base", a.base), res.get("R", a.R))
        log_row("grasp drill rep",
                f"base {res.get('base')}, R {res.get('R')}, rot {res.get('rot')}",
                "pass", "held by wiggle test, then placed at floor height")
        time.sleep(1.0)

    print(f"\n=== {held} held of {a.reps} "
          f"({'criterion met' if held >= 4 else 'criterion is 4 of 5'})")
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
