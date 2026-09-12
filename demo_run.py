#!/usr/bin/env python3
"""The bar-into-the-bag demo, in one continuous run: grasp -> carry -> place -> back off.

    cd /home/astra/robotics && python3 demo_run.py

Filmed on 2026-09-12. Typical timing: ~85 s end to end (grasp ~18 s, drive ~13 s,
position check ~10 s, placement ~25 s, return ~12 s).

WHAT IT ASSUMES - it is a demo script, not a general errand:
  - the arm is at home, jaws empty;
  - the bar lies on the floor right in front, at the forward pose R~158 (where the
    previous run's "put it down" left it), and stands roughly ALONG the car's axis -
    a bar lying across the jaws needs the wrist rotated, which this does not do;
  - the bag is straight ahead, ~1 m away, just past the balcony door track;
  - rig.GRASP_PIXEL is current. Re-measure after any reboot or knock:
        python3 -c "import pick_eye as pe; print(pe.measure_grasp_pixel())"

THE DOOR TRACK IS CROSSED WITHOUT THE GATE, ON PURPOSE. The preflight gate blocks at the
track every time (sonar ~20 cm against the 25 cm a drive needs), and the owner's standing
instruction for the demo was "Обходи порог всегда! У нас демо!" - so the drives here call
car.py directly. Outside the demo, use gated_move.sh.

The four things the run is built on - each cost at least one failed take:
  1. Aim at the CLAMP height, not the survey height. The wrist is never exactly vertical,
     so a 117 mm descent moves the apparent aim by ~36 mm. Descend first, correct with the
     BASE only (rotation cannot walk into R_MIN_CHASSIS), close with no vertical move.
  2. Arrival is "fraction of the bag's yellow lying above the bar's tip" <= ~55% from the
     look-down pose. Nothing else on this robot answers "am I there" honestly at this
     range - depth is blind, sonar ranges the door frame.
  3. Mask the top of the frame when finding the bar near the door: the glass reflects the
     bag AND the bar, and a colour mask cannot tell a reflection from the object.
  4. Placement drops fast to z=-30, then feels for the bottom. Contacts seen: -76, -80,
     -75, -69 on the bottom; -56 and -44 on folds. A drop to -45 once landed straight on
     folds, which is why it is -30 and not lower.

Aborts, rather than carrying air, if the clamp comes back empty (stall >= 660).
"""
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np

REPO = os.path.dirname(os.path.abspath(__file__))
os.chdir(REPO)
sys.path.insert(0, REPO)
import pick  # noqa: E402
import pick_eye as pe  # noqa: E402
import rig  # noqa: E402

T0 = time.time()


def say(s):
    print("[%5.1fs] %s" % (time.time() - T0, s), flush=True)


def get(n):
    return int(subprocess.run(["./arm", "get", str(n)], capture_output=True, text=True).stdout.strip())


def frame():
    time.sleep(0.5)                       # a frame grabbed right after a move can be stale
    return pick.frame()


def bar_x():
    hsv = cv2.cvtColor(frame(), cv2.COLOR_BGR2HSV)
    ys, xs = np.nonzero(cv2.inRange(hsv, (95, 40, 90), (140, 255, 255)))
    return xs.mean() if xs.size > 300 else None


def car(*a):
    subprocess.run(["python3", "car.py", "step", *a, "--steer", "center"], capture_output=True)


OFF = math.radians((rig.BASE_FORWARD - 500) / rig.BASE_UNITS_PER_DEG)   # kin frame -> car forward


def fwd(R, z):
    pe.goto_verified(R * math.cos(OFF), R * math.sin(OFF), z)


def above_tip():
    """% of the bag's yellow beyond the held bar's tip, from the look-down pose."""
    fwd(155, 55.0)
    hsv = cv2.cvtColor(frame(), cv2.COLOR_BGR2HSV)
    yl = cv2.inRange(hsv, (18, 110, 60), (38, 255, 255))
    br = cv2.inRange(hsv, (95, 30, 90), (140, 255, 255))
    br[:150, :] = 0                       # the glass reflects the bar up here
    ys, xs = np.nonzero(yl)
    by, bx = np.nonzero(br)
    if not (xs.size and bx.size):
        return None, xs.size
    return 100.0 * (yl[:by.min(), :].sum() // 255) / xs.size, xs.size


# 1. GRASP - aim at the clamp height, base only, close without a vertical move
pe.arm_step("1:156", 500)
fwd(158, -78.0)
for _ in range(3):
    bx = bar_x()
    if bx is None:
        break
    dx = rig.GRASP_PIXEL[0] - bx
    if abs(dx) <= 10:
        break
    pe.arm_step("6:%d" % (get(6) + int(round(dx / 2.3))), 500)   # ~2.3 px per base unit here
pe.arm_step("1:700", 1000)
time.sleep(0.6)
st = get(1)
say("grasp stall %d" % st)
if st >= 660:
    say("EMPTY - stopping, not carrying air")
    raise SystemExit(1)

# 2. LIFT + STOW
x, y, z = pe.current_xyz()
b = math.atan2(y, x)
for R, zz in ((160, -30), (170, 20)):
    pe.goto(R * math.cos(b), R * math.sin(b), zz, 600)
subprocess.run(["python3", "arm.py", "home", "--keep-grip"], capture_output=True)

# 3. DRIVE - one continuous run, over the door track
car("forward", "65", "2.3", "/tmp/demo_drive.jpg")
say("drove; grip %d" % get(1))

# 4. PLACE - top up until the bar is over the middle of the bag, drop, feel, release
f, px = above_tip()
say("above-tip %s%%  bag px %d" % (None if f is None else round(f), px))
for k in range(4):
    if f is not None and f <= 55:
        break
    subprocess.run(["python3", "arm.py", "home", "--keep-grip"], capture_output=True)
    car("forward", "60", "0.5", "/tmp/demo_topup.jpg")
    f, px = above_tip()
    say("top-up %d: above-tip %s%%  bag px %d" % (k, None if f is None else round(f), px))

x, y, z = pe.current_xyz()
pe.goto(x, y, -30.0, 700)                # fast drop; see note 4 in the docstring
x, y, z = pe.current_xyz()
zr, hit = pe.place_until_contact(x, y, rig.GRASP_Z, step_mm=6.0, max_mm=70.0, log=lambda *_: None)
say("contact z=%.1f (floor -75)" % zr)
pe.arm_step("1:515", 900)
time.sleep(0.4)
say("released, grip %d" % get(1))

# 5. OUT, HOME, BACK OFF 80 cm
x, y, z = pe.current_xyz()
b = math.atan2(y, x)
pe.goto(170 * math.cos(b), 170 * math.sin(b), 30.0, 600)
subprocess.run(["python3", "arm.py", "home"], capture_output=True)
car("backward", "65", "2.6", "/tmp/demo_back.jpg")
say("home + backed off 80 cm - DONE")
