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

ARM = "/home/astra/robotics/arm"
OPEN, CLAMP = 156, 640
PITCH_LO, PITCH_HI = 145.0, 195.0
# How far to nudge the arm when measuring the Jacobian. Must stay SMALL: the wrist camera
# is only centimetres from the floor, so it sees up to ~8 px of shift per millimetre - a
# 15 mm probe swung the object clean out of frame and the measurement failed outright.
PROBE_MM = 6.0
GAIN = 1.0                      # full step: the loop is stable now that the pitch is pinned
MAX_STEP_MM = 18.0              # never lunge so far that the target leaves the frame


def arm_step(moves, ms=1200):
    subprocess.run([ARM, "step", moves, "/home/astra/robotics/calib/_tmp.jpg", str(ms)],
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

# ...and colour ALONE is not enough in this workspace. The power strip sits in frame and its
# blue indicator LEDs (plus the bluish sockets) pass the hue test easily. They read as blobs
# of 500-4400 px with aspect 1.0-1.7, and the aim loop happily drove the arm to a LED and
# grasped bare floor (user: "хватит дрочить синие фонари на блоке питания", and earlier "ты
# пытаешься схватить пустой пол"). They are also why the wrist rotation kept switching off:
# wrist_for_bar ignores anything with aspect < 1.8, and a round LED always looks round.
#
# The bar is BIG and ELONGATED; the impostors are small and round. Requiring both separates
# them cleanly on the measured numbers. Set OBJ_REQUIRE_ELONGATED = False for a compact
# target (the USB plug), where this test would reject the real object too - then the LEDs
# have to be excluded some other way (cover them, or move the strip out of frame).
OBJ_MIN_ASPECT = 1.9
OBJ_REQUIRE_ELONGATED = True
# Above this area the blob cannot be an impostor (the biggest LED/socket blob measured was
# 4400 px) so the shape test is skipped. It has to be: once the bar is HELD it fills much of
# the frame and gets clipped, which drags its bounding-box aspect below the threshold. That
# made see() return None during the wiggle test, and a grasp that was probably GOOD got
# scored "NOT held" because the checker had gone blind.
# Must sit ABOVE the largest impostor and BELOW the real bar: measured bar 21377 px, the
# blue glow smeared along the power cable 9549 px. 8000 was tried first and let the glow
# through unchecked - locate() then reported "found" on a frame containing no bar at all.
OBJ_UNAMBIGUOUS_AREA = 15000
# Elongation alone still lets thin GLOW SLIVERS through (measured 1332 px at aspect 2.79 in
# a frame with no bar in it). The bar is never that small in a usable view, so require some
# bulk as well. NOTE: this, OBJ_MIN_ASPECT and OBJ_REQUIRE_ELONGATED are all bar-specific -
# a USB plug is small AND compact and would be rejected by every one of them.
OBJ_SHAPE_MIN_AREA = 2000

# EXTENT/SOLIDITY/SATURATION - ported in from tanggrab.py's bar-orientation filter 2026-07-27
# after see() (used by locate_near/center_grabframe for SEARCH+CENTRING, i.e. BEFORE the arm
# even tries to read orientation) locked onto the power cable's blue glow along the frame's
# left edge - area 5054, aspect 3.35, comfortably elongated enough to pass the check above -
# and reported "found" with high confidence. It only ever got caught downstream because a
# human looked at the sent photo; tanggrab.measure()'s stricter filter (which already had
# these three checks) would have rejected it too, IF centring had gotten that far without
# aiming the whole arm at the cable first. Search must not lock onto it in the first place.
OBJ_MIN_EXTENT = 0.50      # contour area / minAreaRect area - the cable's glow is diffuse/ragged
OBJ_MIN_SOLIDITY = 0.65    # contour area / convex-hull area
OBJ_MAX_SATURATION = 190   # upper bound: this bar is pencil-pale, glow/LEDs are vivid. Also
                           # distance-dependent (closer views of the SAME bar read higher) -
                           # see tanggrab.BAR_MAX_SATURATION's note before tightening this.


def _mean_saturation(c, hsv):
    x, y, w, h = cv2.boundingRect(c)
    if w <= 0 or h <= 0:
        return 255.0
    mask = np.zeros(hsv.shape[:2], np.uint8)
    cv2.drawContours(mask, [c], -1, 255, -1)
    vals = hsv[:, :, 1][mask > 0]
    return float(vals.mean()) if vals.size else 255.0


def _bar_shaped(c, hsv):
    """True if contour c's shape+colour match the bar, not a round LED or a cable glow.

    The one shared predicate for both see() (search/centring) and tanggrab.measure()
    (orientation) - kept in one place after the two drifted apart and see() alone let the
    cable glow through (it only checked aspect, not extent/solidity/saturation)."""
    a = cv2.contourArea(c)
    if a < OBJ_SHAPE_MIN_AREA:
        return False
    (_, _), (w, h), _ = cv2.minAreaRect(c)
    if max(w, h) / max(1.0, min(w, h)) < OBJ_MIN_ASPECT:
        return False
    extent = a / max(1.0, w * h)
    if extent < OBJ_MIN_EXTENT:
        return False
    hull = cv2.convexHull(c)
    if a / max(1.0, cv2.contourArea(hull)) < OBJ_MIN_SOLIDITY:
        return False
    if _mean_saturation(c, hsv) > OBJ_MAX_SATURATION:
        return False
    return True


def see():
    """Where the object is in the image right now (wrist camera)."""
    img = pick.frame()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, OBJ_LO, OBJ_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = []
    for c in cnts:
        a = cv2.contourArea(c)
        if not (OBJ_MIN_AREA <= a <= OBJ_MAX_AREA):
            continue
        # Above OBJ_UNAMBIGUOUS_AREA the shape/colour test is skipped: a HELD bar fills much
        # of the frame and gets clipped, which drags extent/aspect down and would otherwise
        # blind the wiggle test on a genuinely good grasp.
        if OBJ_REQUIRE_ELONGATED and a < OBJ_UNAMBIGUOUS_AREA and not _bar_shaped(c, hsv):
            continue
        best.append((a, c))
    if not best:
        return None
    best.sort(key=lambda t: -t[0])
    M = cv2.moments(best[0][1])
    if M["m00"] == 0:
        return None
    return M["m10"] / M["m00"], M["m01"] / M["m00"]


# Jaw markers are ~2500-3300 px at grasping distance (rig.py). The window is generous on
# both sides for lighting/partial occlusion, but MUST exclude background objects.
MARKER_AREA_MIN, MARKER_AREA_MAX = 800, 12000
MARKER_AREA_RATIO = 4.0   # the two markers are the same size; a big mismatch means an impostor


def measure_grasp_pixel(restore_grip=OPEN, samples=3):
    """Median of `samples` measurements - see _measure_grasp_pixel_once for the mechanics.

    A single reading is not trustworthy: the FIRST one after a move is regularly an outlier
    (measured (218,255) then (188,354) at the same pose, and (258,268) then (187,350)), which
    silently poisons the aim it feeds. The arm is still settling, and a marker caught
    mid-wobble lands tens of px out. Taking the median of a few costs a second and throws
    the outlier away."""
    pts = []
    for i in range(samples):
        if i:
            time.sleep(0.25)
        p = _measure_grasp_pixel_once(restore_grip if i == samples - 1 else 700)
        if p:
            pts.append(p)
    if not pts:
        return None
    xs = sorted(p[0] for p in pts)
    ys = sorted(p[1] for p in pts)
    rig.GRASP_PIXEL = (xs[len(xs) // 2], ys[len(ys) // 2])
    return rig.GRASP_PIXEL


def _measure_grasp_pixel_once(restore_grip=OPEN):
    """Find where the jaws ACTUALLY close in the image right now, and set rig.GRASP_PIXEL.

    Costs one move. Call it once at the start of any session that grasps - see the long
    note on rig.GRASP_PIXEL for why the stored constant cannot be trusted across mountings,
    and why a stale one fails while the aim loop reports perfect convergence.

    Returns the (x, y) it measured, or None if the two red jaw markers weren't both found
    (in which case rig.GRASP_PIXEL is left alone)."""
    arm_step(f"1:700", 900)                      # close the empty jaws
    img = pick.frame()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = (cv2.inRange(hsv, np.array([0, 60, 50]), np.array([15, 255, 255])) |
         cv2.inRange(hsv, np.array([165, 60, 50]), np.array([180, 255, 255])))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in cnts:
        a = cv2.contourArea(c)
        # AREA WINDOW, and it is load-bearing. The jaw markers cover ~2500-3300 px from a few
        # centimetres away (rig.py). Without an upper bound, any big red/orange thing in the
        # BACKGROUND wins: with a sofa in view a 34000 px blob was being taken for a marker,
        # and since it moves through the frame as the arm swings, the "closing point" wandered
        # ~200 px between poses and ~90 px at the SAME pose. That looked exactly like a loose
        # camera mount and sent me hunting a hardware fault that did not exist.
        if not (MARKER_AREA_MIN <= a <= MARKER_AREA_MAX):
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        blobs.append((a, M["m10"] / M["m00"], M["m01"] / M["m00"]))
    arm_step(f"1:{restore_grip}", 700)

    # Take the two BIGGEST qualifying blobs and use their midpoint. Do NOT split the frame
    # down the middle and demand one marker per half: the markers only straddle the centre
    # in some poses. Measured live with both jaw markers plainly visible at x=313 and x=52,
    # i.e. both LEFT of centre - the half-split found nothing on the right and the whole
    # measurement failed, silently falling back to the stale stored constant.
    blobs.sort(key=lambda b: -b[0])
    if len(blobs) < 2:
        return None
    a, b = blobs[0], blobs[1]
    # Sanity: the two markers are the same physical size, so wildly unequal areas mean one of
    # them is not a marker.
    if a[0] > MARKER_AREA_RATIO * b[0]:
        return None
    rig.GRASP_PIXEL = ((a[1] + b[1]) / 2, (a[2] + b[2]) / 2)
    return rig.GRASP_PIXEL


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


def measure_jacobian(x, y, z, see_fn=None):
    """dPixel/dMillimetre, measured by nudging the arm and watching the object slide.

    Beats deriving it from a calibrated camera: it costs two moves, needs no calibration,
    and it silently absorbs the arm's own inaccuracy - what we get is the mapping from
    COMMANDS to pixels, which is the mapping we actually steer with."""
    see = see_fn or globals()["see"]
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


def servo(x, y, z, iters, tol_px, label="", see_fn=None):
    """Steer the object onto GRASP_PIXEL at height z. Returns the final (x, y), or None.

    The claw is rigid to the camera, so GRASP_PIXEL is the right target at ANY height -
    which is what makes a coarse pass up high possible.

    see_fn overrides how the target point is found (default: see(), the blob centroid).
    Pass tanggrab.see_centre to aim at an elongated object's MIDDLE instead - the centroid
    drifts toward whichever end is more visible, which is what makes the jaws land near a
    bar's edge."""
    see = see_fn or globals()["see"]
    J = measure_jacobian(x, y, z, see_fn=see)
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
            Jn = measure_jacobian(x, y, z, see_fn=see)
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

    cv2.imwrite("/home/astra/robotics/calib/pre_grasp_eye.jpg", pick.frame())
    arm_step(f"1:{CLAMP}", 900)
    print("  сомкнул. (Показание клешни ничего не доказывает — проверяем подъёмом.)")
    goto(x, y, rig.GRASP_Z + 90.0, 1500)      # lift, jaws stay shut
    return True


if __name__ == "__main__":
    pick_object()
