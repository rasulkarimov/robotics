#!/usr/bin/env python3
"""Grasping with the camera on the wrist ("eye-in-hand"). Replaces pick3d.py.

WHY THIS IS SO MUCH SIMPLER THAN WHAT CAME BEFORE
The camera and the jaws are bolted to the same wrist, so the claw sits at a FIXED pixel
no matter how the arm moves (measured: 0.4 px of variation across wildly different
poses). Grasping therefore reduces to: "steer the object onto that pixel, then close."

Gone, and not missed:
  - the 3D camera model (cv2.calibrateCamera over a swept volume)
  - the homography, and the calibrated zone that objects kept wandering out of
  - the parallax correction, and the jaw-closing offset that defeated every early attempt
  - hunting for the claw in each frame - it is always in the same place
And it now works anywhere the arm can reach, instead of inside one small fitted hull.

The one thing the camera cannot give us is DEPTH along its own line of sight, so the
height is not servoed: we simply descend to rig.GRASP_Z, which was measured by hand as
the height at which the CLOSING jaws meet the floor exactly.

HOW THE ARM IS STEERED
The pixel error is converted into a millimetre move by an image Jacobian measured on the
spot: nudge the arm a known distance, see how far the object slid across the frame. Two
nudges give the 2x2 matrix. No camera calibration, and it self-corrects for the fact that
the arm does not actually go where it is told.
"""
import math
import subprocess
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, "/home/astra/tools")
import calib
import kin
import pick
import rig

ARM = "/home/astra/tools/arm"
OPEN, CLAMP = 156, 640
PITCH_LO, PITCH_HI = 145.0, 195.0
# How far to nudge the arm when measuring the Jacobian. Must stay SMALL: the wrist camera
# is only centimetres from the floor, so it sees up to ~8 px of shift per millimetre - a
# 15 mm probe swung the object clean out of frame and the measurement failed outright.
PROBE_MM = 6.0
GAIN = 1.0                      # full step: the loop is stable now that the pitch is pinned
MAX_STEP_MM = 18.0              # never lunge so far that the target leaves the frame


def arm_step(moves, ms=1200):
    subprocess.run([ARM, "step", moves, "/home/astra/tools/calib/_tmp.jpg", str(ms)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)


# FIX the approach angle - do not let the IK choose it.
#
# ik_search picks any pitch in a range, so as the arm translates the wrist quietly ROTATES
# too, and that rotation cancels much of the translation in the image. The result is a
# Jacobian that is nearly singular - one direction of arm motion barely moves the object on
# screen at all - and steering by its inverse amplifies noise until the loop diverges.
#
# Measured conditioning of the image Jacobian:
#     free pitch (ik_search) : 34    <- nearly blind in one direction
#     fixed 175 deg          :  6.8
#     fixed 185 deg          :  1.8  <- both axes respond clearly
# A NARROW band, not a single value: pinning the pitch exactly gives the best-conditioned
# Jacobian but leaves the IK too little freedom, and it starts declaring reachable points
# unreachable. A few degrees of slack restores reachability while keeping the wrist
# essentially still, so the image still responds cleanly to translation.
FIXED_PITCH = 185.0
# PITCH_BAND widened 7->22 on 2026-07-16: ik_search still PREFERS FIXED_PITCH (steep, best for
# seeing the floor), so close objects are unaffected — but a wider band lets the wrist LEAN
# (elbow straighter) to reach objects past R178 at the HIGH pose, which the ±7 band refused.
# The claw stays a fixed pixel (camera+jaws share the wrist), so GRASP_PIXEL holds when leaning.
PITCH_BAND = 22.0


def goto(x, y, z, ms=1200):
    sol = kin.ik_search(x, y, z, pitch_lo=FIXED_PITCH - PITCH_BAND,
                        pitch_hi=FIXED_PITCH + PITCH_BAND, prefer=FIXED_PITCH)
    if not sol:
        return False
    arm_step(",".join(f"{j}:{sol[j]}" for j in (6, 5, 4, 3)), ms)
    return True


# Object detection for the WRIST view. pick.find_object() cannot be reused: it still gates
# on the old homography's calibrated hull (meaningless once the camera left its tripod) and
# caps the blob at 6000 px - but from a few centimetres away the block fills a third of the
# frame. Both filters silently reported "no object" while it sat plainly in the picture.
# The target is BLUE. It used to be a plain white block, and that could never work: the
# wrist camera sees the whole ROOM when the arm is raised - walls, curtains, daylight - and
# "biggest white blob" happily locked onto a bright patch of background. The servo then
# dutifully drove the claw AWAY from the object, towards the curtains ("ты был над
# предметом, потом свернул направо где его не было"). We had already proved white is
# hopeless here once, by measuring the sunlit floor as BRIGHTER than a white object, and
# then went back to a white block anyway.
#
# Colour fixes it outright: exactly one blue blob in the frame, nothing else close.
# It must not be RED - that is taken by the jaw markers.
OBJ_LO = np.array([95, 80, 50])
OBJ_HI = np.array([130, 255, 255])
OBJ_MIN_AREA, OBJ_MAX_AREA = 400, 90000


def see():
    """Where the object is in the image right now (wrist camera)."""
    img = pick.frame()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, OBJ_LO, OBJ_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, _, stats, cent = cv2.connectedComponentsWithStats(m)
    best = [(stats[i, cv2.CC_STAT_AREA], i) for i in range(1, n)
            if OBJ_MIN_AREA <= stats[i, cv2.CC_STAT_AREA] <= OBJ_MAX_AREA]
    if not best:
        return None
    best.sort(key=lambda t: -t[0])
    c = cent[best[0][1]]
    return float(c[0]), float(c[1])


def held(before, tol=40.0):
    """A grasp is proved by the object NOT MOVING in the image while the arm does.

    The wrist camera makes this trivial: camera and jaws are one rigid body, so anything
    actually in the jaws is nailed to the same pixel however the arm swings, while
    anything left on the floor slides across the frame. Far more honest than the gripper
    reading, which sits around 620-630 whether it is holding a thin object or nothing."""
    p = see()
    if p is None or before is None:
        return False
    return math.dist(p, before) < tol


def measure_jacobian(x, y, z):
    """dPixel/dMillimetre, measured by nudging the arm and watching the object slide.

    Beats deriving it from a calibrated camera: it costs two moves, needs no calibration,
    and it silently absorbs the arm's own inaccuracy - what we get is the mapping from
    COMMANDS to pixels, which is the mapping we actually steer with."""
    p0 = see()
    if p0 is None:
        return None
    cols = []
    for dx, dy in ((PROBE_MM, 0.0), (0.0, PROBE_MM)):
        if not goto(x + dx, y + dy, z, 900):
            return None
        p = see()
        if p is None:
            goto(x, y, z, 900)
            return None
        cols.append([(p[0] - p0[0]) / PROBE_MM, (p[1] - p0[1]) / PROBE_MM])
    goto(x, y, z, 900)
    J = np.array(cols).T                      # 2x2: pixel shift per mm of command
    if abs(np.linalg.det(J)) < 1e-6:
        return None
    return J


def servo(x, y, z, iters, tol_px, label=""):
    """Steer the object onto GRASP_PIXEL at height z. Returns the final (x, y), or None.

    The claw is rigid to the camera, so GRASP_PIXEL is the right target at ANY height -
    which is what makes a coarse pass up high possible."""
    J = measure_jacobian(x, y, z)
    if J is None:
        return None
    prev = None
    for i in range(iters):
        p = see()
        if p is None:
            print(f"  {label}: ПОТЕРЯЛ ПРЕДМЕТ ИЗ ВИДА")
            return None
        err = np.array([rig.GRASP_PIXEL[0] - p[0], rig.GRASP_PIXEL[1] - p[1]])
        d = float(np.linalg.norm(err))
        print(f"  {label} итерация {i}: ошибка {d:.0f} px")
        if d <= tol_px:
            return x, y
        if prev is not None and d > prev:
            Jn = measure_jacobian(x, y, z)
            if Jn is not None:
                J = Jn
        prev = d
        mv = GAIN * solve(J, err)

        # CLAMP THE STEP. Unbounded, a large pixel error produces a large lunge, the
        # object swings clean out of the narrow field of view, and the loop is left with
        # nothing to steer by (seen: 219 -> 126 -> 217 px -> target lost). Better several
        # short hops that keep the object in sight than one leap that loses it.
        n = float(np.linalg.norm(mv))
        if n > MAX_STEP_MM:
            mv = mv * (MAX_STEP_MM / n)

        nx, ny = x + float(mv[0]), y + float(mv[1])
        if not kin.reachable(nx, ny, z) or not goto(nx, ny, z, 900):
            # Do NOT pretend this succeeded. Returning the position anyway let the caller
            # clamp on thin air and then solemnly carry an imaginary object around - the
            # third time this exact "fail quietly, carry on" bug has bitten. Report the
            # residual error and let the caller decide.
            print(f"  {label}: упёрся в предел зоны (ошибка {d:.0f} px)")
            return None
        x, y = nx, ny
    print(f"  {label}: не сошёлся за {iters} итераций (ошибка {d:.0f} px)")
    return None


def solve(J, err, lam=0.8):
    """Pixel error -> millimetre move, via DAMPED least squares rather than a plain inverse.

    The measured Jacobian is sometimes badly conditioned - one run came out at 1.19 vs
    8.13 px/mm, i.e. one direction of arm motion barely shifts the object in the image.
    Inverting that amplifies the detector's noise enormously, and the loop diverges
    (13.9 -> 27.3 -> 70.3 px, straight off the target). Damping bounds the correction in
    the ill-conditioned direction instead of trusting it."""
    JT = J.T
    return JT @ np.linalg.solve(J @ JT + (lam ** 2) * np.eye(2), err)


# Tolerance tied to PHYSICS, not to a tidy-looking number. The open jaws are wide, and
# grasps succeed reliably with 30-50 px of residual error - so chasing 6-8 px was chasing
# nothing, and worse, it made me read three SUCCESSFUL grasps as failures and "fix" working
# code. Stop when the grasp is already assured.
def pick_object(iters=8, tol_px=25.0):
    # Start high enough that the claw cannot touch anything while we look around.
    approach_z = rig.GRASP_Z + 70.0

    p = see()
    if p is None:
        print("предмет не вижу")
        return False
    print(f"предмет в кадре: ({p[0]:.0f},{p[1]:.0f}), "
          f"целевой пиксель клешни: ({rig.GRASP_PIXEL[0]:.0f},{rig.GRASP_PIXEL[1]:.0f})")

    arm_step(f"1:{OPEN}", 700)

    # Steer at the grasp height: the mapping pixel<->mm depends on how far the camera is
    # from the floor, so aligning high up and then dropping would land somewhere else.
    # Start pointing straight ahead of the car (rig.BASE_FORWARD), not along the model's
    # x axis - the two are 7.5 degrees apart, and the servo's own centre is not "forward".
    fwd = kin.s2a(rig.BASE_FORWARD, 6)
    x, y = 150.0 * math.cos(fwd), 150.0 * math.sin(fwd)
    if not goto(x, y, rig.GRASP_Z, 1400):
        print("не могу встать на высоту захвата")
        return False

    J = measure_jacobian(x, y, rig.GRASP_Z)
    if J is None:
        print("не смог измерить связь пиксели<->миллиметры (предмет пропал из вида?)")
        return False
    print(f"измерил: сдвиг на 1 мм двигает предмет на "
          f"{np.linalg.norm(J[:,0]):.2f}/{np.linalg.norm(J[:,1]):.2f} px")

    prev_d = None
    for i in range(iters):
        p = see()
        if p is None:
            # NEVER carry on quietly here. An earlier version just `break`ed, so the loop
            # fell out, the jaws clamped on thin air, and the whole demo then solemnly
            # carried an imaginary object around. Losing the target is a failure - say so.
            print("  ПОТЕРЯЛ ПРЕДМЕТ ИЗ ВИДА — не смыкаю, это был бы захват воздуха")
            return False
        err = np.array([rig.GRASP_PIXEL[0] - p[0], rig.GRASP_PIXEL[1] - p[1]])
        d = float(np.linalg.norm(err))
        print(f"  итерация {i}: предмет=({p[0]:.0f},{p[1]:.0f}) ошибка={d:.1f} px")
        if d <= tol_px:
            break

        # If the error GREW, the Jacobian we are steering by is lying to us - it is only
        # valid near where it was measured, and we have moved. Re-measure rather than
        # press on: pressing on is how a run went 13.9 -> 27.3 -> 70.3 px and drove the
        # claw right off the object.
        if prev_d is not None and d > prev_d:
            print("    (ошибка выросла — перемеряю якобиан)")
            Jn = measure_jacobian(x, y, rig.GRASP_Z)
            if Jn is not None:
                J = Jn
        prev_d = d

        move = GAIN * solve(J, err)
        nx, ny = x + float(move[0]), y + float(move[1])
        r = math.hypot(nx, ny)
        rmax = rig.max_floor_radius()
        if not (rig.MIN_FLOOR_RADIUS <= r <= rmax):
            print(f"  коррекция уводит на радиус {r:.0f} мм — вне рабочей зоны "
                  f"({rig.MIN_FLOOR_RADIUS:.0f}..{rmax:.0f})")
            return False
        if not goto(nx, ny, rig.GRASP_Z, 900):
            print("  коррекция недостижима")
            return False
        x, y = nx, ny

    cv2.imwrite("/home/astra/tools/calib/pre_grasp_eye.jpg", pick.frame())
    arm_step(f"1:{CLAMP}", 900)
    print("  сомкнул. (Показание клешни ничего не доказывает — проверяем подъёмом.)")
    goto(x, y, rig.GRASP_Z + 90.0, 1500)      # lift, jaws stay shut
    return True


if __name__ == "__main__":
    pick_object()
