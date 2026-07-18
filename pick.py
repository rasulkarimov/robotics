#!/usr/bin/env python3
"""Autonomous pick: see an object -> compute where it is in mm -> grasp it.

This is what all the other tools were built for. The chain is:
    camera pixel --(calib.py homography)--> floor mm --(kin.py inverse kinematics)-->
    joint angles --(arm.py)--> the claw closes on the thing.

No teaching, no hand-guiding, no hardcoded poses: the object can be anywhere inside the
calibrated zone. Contrast with the earlier `arm.py replay grab`, which only ever worked
for one object in one spot because it was a recorded trajectory.

WHY IT ONLY WORKS INSIDE THE ZONE: the homography is fitted over the region the claw
swept during calibration (calib/points.json). Outside it the transform extrapolates and
lies - which is exactly how an object 5 cm out of the calibrated area got "grasped" into
thin air. `--check` refuses to move if the object is outside the fitted hull.

The object detector is deliberately dumb (bright, low-saturation blob = the white mains
adapter on a beige floor). It is not a general object detector and does not pretend to
be; swap in a real one when you need arbitrary objects.
"""
import json
import math
import subprocess
import sys
import time
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, "/home/astra/tools")
import calib
import kin

ARM = "/home/astra/tools/arm"
SNAPSHOT_URL = "http://127.0.0.1:8090/?action=snapshot"

OPEN, CLAMP = 156, 640          # gripper: clamp toward fully-closed so it grips with
                                # force and stalls on the object, rather than just
                                # touching it at the measured width
HOVER_Z = 80.0                  # approach height, well clear of the object
GRASP_Z = calib.CAL_Z           # grasp ON the floor plane - the one plane the homography
                                # is actually exact for, and the plane the object sits on
PITCH_LO, PITCH_HI = calib.PITCH_LO, calib.PITCH_HI

# The target is a white block. Detecting white on a beige floor is genuinely marginal -
# measured on this scene, sunlit floor reached V=170 at the same saturation as a white
# object, so "biggest bright blob" once aimed the arm at the middle of the floor, and
# the blob's shape shifted with the arm's shadow, making the phantom target appear to
# move (a detection bug that looked exactly like a servo bug). It only works here
# because of the area bounds AND the zone check below; a coloured target is far safer.
OBJ_LO = np.array([0, 0, 170])
OBJ_HI = np.array([180, 60, 255])
MIN_AREA, MAX_AREA = 500, 6000

# How far outside the fitted zone we will still attempt a grasp. Strictly, the
# homography extrapolates out here and its answer is not to be trusted - but the servo
# loop converges on the IMAGE, using the homography only as a local scale factor, so it
# tolerates a poor starting estimate. A hard limit still applies: far outside, even the
# scale factor is wrong and the loop can walk away from the target.
ZONE_MARGIN_PX = 45.0


def frame():
    time.sleep(0.35)
    with urllib.request.urlopen(SNAPSHOT_URL, timeout=5) as r:
        urllib.request.urlopen(SNAPSHOT_URL, timeout=5).read()   # drop stale buffered frame
        return cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)


def find_object(img=None):
    """The yellow target: a coloured blob, larger than a jaw marker, inside the
    calibrated zone. All three conditions matter - see OBJ_LO/MIN_AREA."""
    img = frame() if img is None else img
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, OBJ_LO, OBJ_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, _, stats, cent = cv2.connectedComponentsWithStats(m)
    best = [(stats[i, cv2.CC_STAT_AREA], i) for i in range(1, n)
            if MIN_AREA <= stats[i, cv2.CC_STAT_AREA] <= MAX_AREA
            and in_zone(cent[i][0], cent[i][1])]
    if not best:
        return None
    best.sort(key=lambda t: -t[0])
    i = best[0][1]

    # Aim at where the object MEETS THE FLOOR, not at its centroid. The homography is
    # exact on the floor plane and nowhere else; an object's centroid sits above that
    # plane, so its pixel maps to a floor point that isn't actually under it - which is
    # how the claw kept arriving perfectly on-target in the image and still missing.
    # The bottom edge of the blob is its floor contact, and that IS on the plane.
    cx = cent[i][0]
    base_y = stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] - 2
    return float(cx), float(base_y)


def zone_margin(px, py):
    """Signed distance to the fitted zone, in pixels (positive = inside)."""
    pts = json.load(open(calib.POINTS_JSON))
    hull = cv2.convexHull(np.array([p["px"] for p in pts], dtype=np.float32))
    return cv2.pointPolygonTest(hull, (float(px), float(py)), True)


def in_zone(px, py):
    return zone_margin(px, py) >= -ZONE_MARGIN_PX


def arm_step(moves, ms=1200):
    subprocess.run([ARM, "step", moves, "/home/astra/tools/calib/_tmp.jpg", str(ms)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)


def gripper():
    out = subprocess.run([ARM, "get", "1"], capture_output=True, text=True).stdout
    return int(out.strip())


def goto(x, y, z, ms=1300):
    sol = kin.ik_search(x, y, z, pitch_lo=PITCH_LO, pitch_hi=PITCH_HI,
                        prefer=(PITCH_LO + PITCH_HI) / 2)
    if not sol:
        return False
    arm_step(",".join(f"{j}:{sol[j]}" for j in (6, 5, 4, 3)), ms)
    return True


def servo_onto(x, y, target_px, iters=4, tol_px=4.0):
    """Closed-loop visual servoing: nudge the claw until its markers sit on the object
    IN THE IMAGE, then trust that rather than the model.

    Open-loop (pixel -> mm -> move) missed, and it always will: the homography is fitted
    on the claw markers at CAL_Z, but an object's pixel is the centroid of its visible
    top, at a different height - so parallax shifts it. Add arm sag and the offset
    between "marker midpoint" and "where the jaws actually meet", and the model is
    systematically off by more than the object is wide.

    Servoing cancels ALL of that at once, because the marker midpoint is literally the
    centre of the gripper opening (one marker per jaw). Drive that onto the object's
    pixel and the jaws close around it - no matter what the model thinks.

    The homography still earns its keep as the local Jacobian: it converts a pixel error
    into roughly the right millimetre correction, which is all a servo loop needs."""
    for i in range(iters):
        # Re-find the OBJECT every iteration, not just the claw. The descending claw can
        # nudge the target, and servoing against the pixel it was first seen at then
        # drives the jaws confidently onto the spot the object has just left - which is
        # exactly what the debug frames showed: a perfectly converged claw closing on
        # bare floor.
        fresh = find_object()
        if fresh is not None:
            target_px = fresh

        claw = calib.find_claw()
        if claw is None:
            print("  маркеры клешни не видны — не могу навестись")
            return None
        ex, ey = target_px[0] - claw[0], target_px[1] - claw[1]
        err = math.hypot(ex, ey)
        print(f"  итерация {i}: клешня=({claw[0]:.0f},{claw[1]:.0f}) "
              f"предмет=({target_px[0]:.0f},{target_px[1]:.0f}) ошибка={err:.1f} px")
        if err <= tol_px:
            return x, y
        # local Jacobian from the homography: where would the claw need to be, in mm,
        # to land on the object's pixel?
        tx, ty = calib.pixel_to_floor(*target_px)
        cx, cy = calib.pixel_to_floor(*claw)
        x += tx - cx
        y += ty - cy
        if not kin.reachable(x, y, GRASP_Z):
            print("  коррекция уводит за пределы досягаемости")
            return None
        goto(x, y, GRASP_Z, 900)
    return x, y


def pick(dry_run=False, grasp_z=None):
    global GRASP_Z
    if grasp_z is not None:
        GRASP_Z = float(grasp_z)
    px = find_object()
    if px is None:
        print("предмет не найден")
        return False
    x, y = calib.pixel_to_floor(*px)
    r = math.hypot(x, y)
    zone = in_zone(*px)
    reach = kin.reachable(x, y, GRASP_Z)
    print(f"вижу предмет: пиксель ({px[0]:.0f},{px[1]:.0f}) -> x={x:.1f} y={y:.1f} мм (r={r:.0f})")
    print(f"  в откалиброванной зоне: {'да' if zone else 'НЕТ'} | достижим: {'да' if reach else 'НЕТ'}")
    if not zone:
        print("  ОТКАЗ: вне зоны — гомография там экстраполирует, промахнусь")
        return False
    if not reach:
        print(f"  ОТКАЗ: вне досягаемости (максимум {kin.max_reach(GRASP_Z):.0f} мм)")
        return False
    if dry_run:
        return True

    arm_step(f"1:{OPEN}", 600)
    if not goto(x, y, HOVER_Z):
        print("  не могу зайти на высоту подхода")
        return False
    if not goto(x, y, GRASP_Z):
        print("  не могу опуститься к предмету")
        return False

    out = servo_onto(x, y, px)            # close the loop on the image
    if out is None:
        return False
    x, y = out

    # Snapshot the instant BEFORE the jaws close. Without it a failed grasp is a
    # mystery - you see the aftermath and can't tell whether the claw was beside the
    # object, above it, or gripping and dropping it.
    img = frame()
    claw = calib.find_claw(img)
    if claw:
        cv2.drawMarker(img, (int(claw[0]), int(claw[1])), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
    cv2.drawMarker(img, (int(px[0]), int(px[1])), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
    cv2.imwrite("/home/astra/tools/calib/pre_grasp.jpg", img)
    print("  снимок перед захватом -> calib/pre_grasp.jpg")

    arm_step(f"1:{CLAMP}", 900)
    g = gripper()
    if not goto(x, y, HOVER_Z, 1400):
        return False
    print(f"  клешня после захвата: {g}")
    return True


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "check":
        pick(dry_run=True)
    else:
        z = float(sys.argv[1]) if len(sys.argv) > 1 else None
        pick(grasp_z=z)
