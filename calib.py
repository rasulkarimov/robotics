#!/usr/bin/env python3
"""Camera <-> floor calibration (homography) for the xArm rig.

THE TRICK: the arm calibrates itself. No checkerboard, no ruler. kin.py knows exactly
where the grasp point is in millimetres, so we drive the claw over a grid of KNOWN
coordinates, see where it lands in the image, and fit pixel->mm from those pairs.

FINDING THE CLAW - three attempts, only the third works:
 1. Threshold on darkness: useless, the arm/car/cables merge into one dark blob.
 2. Open/close the gripper and diff the frames: right idea, but when the claw is near
    the floor the jaws push against it and rock the WHOLE arm, so the biggest moving
    blob is the arm's body, not the jaws (this poisoned an entire calibration run -
    every point landed near the base). Even clear of the floor it was noisy: the claw
    is dark, self-shadowing, and its "moved region" centroid wandered 10-25 px.
 3. Two yellow markers stuck on the jaws (user added them): trivially detectable by
    colour, and their MIDPOINT is the grasp centre and is invariant to the gripper
    opening. Repeatability measured at 0.2 px, vs 10-25 px for (2). ~100x better.

APPROACH ANGLE: kept inside PITCH_LO..PITCH_HI. Outside that band the wrist rolls the
markers out of the camera's view, and a fixed angle is over-constrained (it makes half
the workspace "unreachable"), so we search within the band.

INVALIDATED BY: moving the camera, or moving the arm relative to it. Both happened
repeatedly. Re-run `collect` then `fit` after either, and re-check `holdout_median_mm`.
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
import kin

ARM = "/home/astra/robotics/arm"
SNAPSHOT_URL = "http://127.0.0.1:8090/?action=snapshot"
CALIB_DIR = "/home/astra/robotics/calib"
POINTS_JSON = f"{CALIB_DIR}/points.json"
HOMOGRAPHY_JSON = f"{CALIB_DIR}/homography.json"

# CALIBRATE ON THE FLOOR ITSELF, with the claw touching it.
#
# A homography is exact for exactly ONE plane. Calibrating with the claw hovering (the
# old CAL_Z=30) fitted a plane 30 mm ABOVE the floor - but the object lies ON the floor,
# so its pixel and the claw's pixel could agree perfectly while the two were still
# centimetres apart along the camera's viewing ray. That is why every servo run drove
# the error down to 2-3 px and then closed on thin air.
#
# Put the claw ON the floor and the fitted plane IS the floor, so agreement in the image
# becomes agreement in reality. Measured: contact happens around commanded z=10 (below
# that the claw stops descending), so z=5 presses lightly everywhere in the zone despite
# the arm sagging more at longer reach. Any deflection from that contact is absorbed by
# the fit, because what we calibrate is "commanded (x,y) -> pixel the claw ACTUALLY
# reaches", which is exactly the map a grasp needs.
#
# Safe to touch down now: the old floor-contact problem (jaws pushing the floor rocked
# the whole arm) came from detecting the claw by open/close differencing. Colour markers
# need no gripper motion at all.
#
# WHY NOT ACTUALLY TOUCH DOWN: kin.py's zero is optimistic - stepping the claw down and
# watching the markers, it falls freely at ~0.77 px/mm, slows below z=5, and has stopped
# entirely by z=-25 (0.2 px per 5 mm), i.e. resting on the floor and merely flexing. So
# z=5 leaves the claw ~14 mm up, which the user also spotted by eye.
#
# But calibrating with real contact (z=-20) made the fit WORSE, 4.6 -> 8.5 mm: pressing
# on the floor bends the arm, by an amount that varies with reach, so the commanded (x,y)
# stops being the truth and the "known" half of every calibration pair is corrupted.
#
# So: hover just clear of the floor (free, undistorted), and kill the remaining parallax
# on the other side instead - pick.py aims at the object's floor-contact pixel rather
# than its centroid, which puts the TARGET on the plane even if the claw is a little
# above it.
CAL_Z = 5.0
PITCH_LO, PITCH_HI = 145.0, 175.0  # band where the markers stay visible
OPEN = 156

# Jaw markers are RED. They were yellow, but yellow/orange also appears on the cabling
# and elsewhere in frame, and the detector kept jumping to those; red is unique here.
# (Red straddles the hue wraparound, hence two ranges.)
RED_RANGES = [((0, 110, 60), (10, 255, 255)), ((170, 110, 60), (180, 255, 255))]
# Blob size bounds for a jaw marker. These are VIEW-DEPENDENT: from the old external
# camera the markers covered ~50 px, but with the camera on the WRIST they are inches
# away and cover ~2500-3300 px - so the old 600 px ceiling silently discarded the very
# markers it was meant to find. Keep the ceiling generous; the target object is a
# different colour anyway, so there is nothing to confuse them with.
MIN_BLOB, MAX_BLOB = 20, 9000
# Left-edge cutoff. This existed only for the old EXTERNAL camera, where the car and its
# cabling sat on the left of the frame and formed their own close-together pairs that the
# closest-pair rule below mistook for jaws. With the camera now on the WRIST the jaws are
# at the bottom of the frame - the left one lands around x=85 - so the same cutoff quietly
# threw the real markers away. Set to 0 for the wrist camera.
ROI_X_MIN = 0


def grab_frame():
    with urllib.request.urlopen(SNAPSHOT_URL, timeout=5) as r:
        return cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)


def settled_frame():
    """mjpg-streamer serves a buffered frame, so a snapshot right after a move can still
    show the PREVIOUS pose. Wait, then throw one frame away before keeping the next."""
    time.sleep(0.4)
    grab_frame()
    return grab_frame()


# Max pixel separation of the two jaw markers. Also VIEW-DEPENDENT: seen from across the
# room they sat ~25 px apart, but the wrist camera looks out from BETWEEN them, so they
# fall on opposite sides of the frame (~314 px apart). A 90 px ceiling rejected them.
MAX_JAW_GAP = 420.0


def find_claw(img=None):
    """Midpoint of the two yellow jaw markers = the grasp point, in pixels.

    The target object is ALSO yellow, so "the two biggest yellow blobs" is not safe: when
    the arm partly occludes the object its visible area shrinks into the marker size range
    and it gets mistaken for a jaw, which silently poisoned a whole calibration run (90th
    percentile error 154 mm). The jaws, however, are always a short distance apart, while
    the object is somewhere else entirely - so pick the CLOSEST PAIR, not the biggest."""
    img = settled_frame() if img is None else img
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = None
    for lo, hi in RED_RANGES:
        part = cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = part if m is None else (m | part)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, _, stats, cent = cv2.connectedComponentsWithStats(m)
    blobs = [i for i in range(1, n)
             if MIN_BLOB <= stats[i, cv2.CC_STAT_AREA] <= MAX_BLOB
             and cent[i][0] >= ROI_X_MIN]
    if len(blobs) < 2:
        return None
    best, best_d = None, MAX_JAW_GAP
    for a in range(len(blobs)):
        for b in range(a + 1, len(blobs)):
            d = math.dist(cent[blobs[a]], cent[blobs[b]])
            if d < best_d:
                best, best_d = (blobs[a], blobs[b]), d
    if best is None:
        return None
    (x1, y1), (x2, y2) = cent[best[0]], cent[best[1]]
    return ((x1 + x2) / 2, (y1 + y2) / 2)


def arm_step(moves, ms=1200):
    subprocess.run([ARM, "step", moves, f"{CALIB_DIR}/_tmp.jpg", str(ms)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)


def collect():
    arm_step(f"1:{OPEN}", 600)
    pairs = []
    # Deliberately a NARROW sweep. Widening it (base 440-800, r 110-230) made the fit
    # much worse - held-out error went from 3 mm to 16 mm, with a 58 mm tail. A
    # homography models a PLANE, and the arm droops further the further it reaches, so
    # over a large area the claw simply isn't on one. Keep the zone small and honest;
    # `pick.py` refuses to grasp outside it rather than trusting an extrapolation.
    for base in (560, 600, 640, 680, 720, 760, 800):
        for r in (120, 150, 180, 210):
            ang = kin.s2a(base, 6)
            x, y = r * math.cos(ang), r * math.sin(ang)
            sol = kin.ik_search(x, y, CAL_Z, pitch_lo=PITCH_LO, pitch_hi=PITCH_HI,
                                prefer=(PITCH_LO + PITCH_HI) / 2)
            if not sol:
                continue
            sol[6] = base
            arm_step(",".join(f"{j}:{sol[j]}" for j in (6, 5, 4, 3)))
            px = find_claw()
            if px is None:
                print(f"  base={base} r={r:3d}: маркеры не видны")
                continue
            pairs.append({"px": list(px), "mm": [x, y], "base": base, "r": r})
            print(f"  base={base} r={r:3d} -> mm=({x:6.1f},{y:6.1f})  px=({px[0]:6.1f},{px[1]:6.1f})")

    json.dump(pairs, open(POINTS_JSON, "w"), indent=2)
    print(f"\nсобрано {len(pairs)} точек -> {POINTS_JSON}")
    subprocess.run([ARM, "home"], stdout=subprocess.DEVNULL)
    return pairs


def fit():
    pairs = json.load(open(POINTS_JSON))
    if len(pairs) < 4:
        print("нужно минимум 4 точки")
        return
    src = np.array([p["px"] for p in pairs], dtype=np.float32)
    dst = np.array([p["mm"] for p in pairs], dtype=np.float32)

    # HELD-OUT validation. Error on RANSAC's own inliers is circular - it picks the
    # points that agree with it, so a small number there proves nothing (an early run
    # cheerfully reported "0.8 mm" while two thirds of the data was garbage). Refit with
    # each point left out and measure the miss on the point that was left out.
    held = []
    for i in range(len(pairs)):
        keep = [j for j in range(len(pairs)) if j != i]
        if len(keep) < 4:
            continue
        Hi, _ = cv2.findHomography(src[keep], dst[keep], cv2.RANSAC, 5.0)
        if Hi is None:
            continue
        p = cv2.perspectiveTransform(src[i].reshape(1, 1, 2), Hi).reshape(2)
        held.append(float(np.linalg.norm(p - dst[i])))

    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
    if H is None:
        print("гомография не найдена")
        return
    inl = int(mask.ravel().sum())
    held = np.array(held)
    print(f"точек: {len(pairs)}, принято RANSAC: {inl}")
    print(f"ОШИБКА НА ОТЛОЖЕННЫХ ТОЧКАХ (честная): медиана {np.median(held):.1f} мм, "
          f"90-й перцентиль {np.percentile(held, 90):.1f} мм")
    json.dump({"H": H.tolist(), "n_points": inl, "cal_z": CAL_Z,
               "holdout_median_mm": float(np.median(held))},
              open(HOMOGRAPHY_JSON, "w"), indent=2)
    print(f"сохранено -> {HOMOGRAPHY_JSON}")


def pixel_to_floor(px, py):
    H = np.array(json.load(open(HOMOGRAPHY_JSON))["H"], dtype=np.float32)
    p = np.array([[[float(px), float(py)]]], dtype=np.float32)
    x, y = cv2.perspectiveTransform(p, H)[0][0]
    return float(x), float(y)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "help"
    if cmd == "collect":
        collect()
    elif cmd == "fit":
        fit()
    elif cmd == "px":
        x, y = pixel_to_floor(float(sys.argv[2]), float(sys.argv[3]))
        print(f"пиксель ({sys.argv[2]},{sys.argv[3]}) -> x={x:.1f} y={y:.1f} мм "
              f"(радиус {math.hypot(x, y):.1f}, достижим: {kin.reachable(x, y, CAL_Z)})")
    else:
        print(__doc__)
        print("usage: calib.py collect | fit | px PX PY")
