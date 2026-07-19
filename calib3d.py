#!/usr/bin/env python3
"""Full 3D camera model (not a homography) for the xArm rig.

WHY THIS REPLACES calib.py
A homography describes exactly ONE plane. Calibrating it on the claw markers and then
aiming at an object lying on the floor meant relating two things at DIFFERENT heights,
so agreement in the image never implied agreement in space: the claw would servo onto
the object to within 1-2 px and still close on bare floor a couple of centimetres to the
side. No amount of offset-tuning fixes that; it is the model that is wrong.

Calibrating a real camera (intrinsics + pose) turns every pixel into a RAY. Intersect
the ray with the floor and you get the object's true position. Parallax doesn't get
compensated - it stops existing.

HOW IT SELF-CALIBRATES
kin.py knows where the claw is in millimetres; the coloured jaw markers say where it is
in pixels. Driving the claw through a VOLUME (several heights, not one plane) yields
non-coplanar 3D<->2D correspondences, which is all cv2.calibrateCamera needs. No
chessboard, no printing, no second camera.

A DELIBERATE CHOICE OF COORDINATES
We feed calibrateCamera the COMMANDED positions, not "true" ones. The arm's real
geometry is imperfect - it sags under its own weight, so commanded z is roughly 25 mm
optimistic (measured: the claw stops descending around commanded z=-25, i.e. that is
where the floor is in command coordinates). Calibrating in command coordinates folds
those errors into the camera model, and what comes out is the map we actually want:
"object seen at this pixel -> command THIS to put the claw on it".
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
import calib   # reuse its marker detector
import kin

ARM = "/home/astra/robotics/arm"
CALIB_DIR = "/home/astra/robotics/calib"
POINTS_JSON = f"{CALIB_DIR}/points3d.json"
MODEL_JSON = f"{CALIB_DIR}/camera3d.json"

FLOOR_Z = -25.0        # the floor, in COMMAND coordinates (see docstring)
PITCH_LO, PITCH_HI = calib.PITCH_LO, calib.PITCH_HI
OPEN = 156


def arm_step(moves, ms=1200):
    subprocess.run([ARM, "step", moves, f"{CALIB_DIR}/_tmp.jpg", str(ms)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)


def goto(x, y, z, ms=1200):
    sol = kin.ik_search(x, y, z, pitch_lo=PITCH_LO, pitch_hi=PITCH_HI,
                        prefer=(PITCH_LO + PITCH_HI) / 2)
    if not sol:
        return False
    arm_step(",".join(f"{j}:{sol[j]}" for j in (6, 5, 4, 3)), ms)
    return True


def collect():
    """Drive the claw through a VOLUME and record (3D command, 2D pixel) pairs."""
    arm_step(f"1:{OPEN}", 600)
    pairs = []
    for z in (0, 25, 50, 75):                      # several heights - the whole point
        for base in (580, 640, 700, 760):
            for r in (130, 170, 210):
                ang = kin.s2a(base, 6)
                x, y = r * math.cos(ang), r * math.sin(ang)
                if not goto(x, y, float(z)):
                    continue
                px = calib.find_claw()
                if px is None:
                    continue
                pairs.append({"xyz": [x, y, float(z)], "px": list(px)})
                print(f"  z={z:3d} base={base} r={r:3d} -> "
                      f"({x:6.1f},{y:6.1f},{z:5.1f}) px=({px[0]:6.1f},{px[1]:6.1f})")
    json.dump(pairs, open(POINTS_JSON, "w"), indent=2)
    print(f"\nсобрано {len(pairs)} точек в объёме -> {POINTS_JSON}")
    subprocess.run([ARM, "home"], stdout=subprocess.DEVNULL)
    return pairs


def _calibrate(pairs, size=(480, 320)):
    obj = np.array([p["xyz"] for p in pairs], dtype=np.float32).reshape(1, -1, 3)
    img = np.array([p["px"] for p in pairs], dtype=np.float32).reshape(1, -1, 2)
    # One view of a non-coplanar target: keep the model simple (pinhole, no distortion)
    # or the solve is under-constrained and will happily fit noise. USE_INTRINSIC_GUESS
    # is mandatory here - OpenCV refuses a non-planar rig without a seeded K.
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST |
             cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 |
             cv2.CALIB_FIX_PRINCIPAL_POINT)
    K0 = np.array([[400.0, 0, size[0] / 2], [0, 400.0, size[1] / 2], [0, 0, 1]])
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        [obj[0]], [img[0]], size, K0, np.zeros(5), flags=flags)
    return rms, K, dist, rvecs[0], tvecs[0]


def _robust_calibrate(pairs, thresh=2.5, rounds=3):
    """Drop points the model cannot explain, then refit.

    Two things poison the raw set. A few marker detections are simply wrong (one point
    reprojected 200 px out). More insidiously, the arm SAGS: residuals rise steadily the
    lower it reaches (median 6 px at z=75, 18 px at z=0), because the commanded position
    - our supposed ground truth - drifts further from reality the more the arm leans out
    under its own weight. A rigid camera model cannot absorb that, so let it discard the
    worst offenders rather than smear the error across every parameter."""
    keep = list(pairs)
    for _ in range(rounds):
        rms, K, dist, rvec, tvec = _calibrate(keep)
        obj = np.array([p["xyz"] for p in keep], dtype=np.float32)
        img = np.array([p["px"] for p in keep], dtype=np.float32)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)
        cut = max(3.0, thresh * np.median(err))
        nxt = [p for p, e in zip(keep, err) if e <= cut]
        if len(nxt) < 12 or len(nxt) == len(keep):
            break
        keep = nxt
    return _calibrate(keep) + (keep,)


def fit():
    pairs = json.load(open(POINTS_JSON))
    if len(pairs) < 12:
        print(f"мало точек ({len(pairs)}), нужно >=12")
        return
    rms, K, dist, rvec, tvec, pairs = _robust_calibrate(pairs)
    print(f"после отбраковки осталось {len(pairs)} точек")

    # HONEST held-out check: refit without each point, then predict THAT point's 3D
    # position from its pixel by intersecting the ray with the plane of its true z.
    # (Reprojection error in pixels flatters the model; what we care about is mm.)
    errs = []
    for i in range(len(pairs)):
        rest = [p for j, p in enumerate(pairs) if j != i]
        try:
            _, Ki, di, rv, tv = _calibrate(rest)
        except cv2.error:
            continue
        gt = np.array(pairs[i]["xyz"])
        got = _pixel_to_plane(pairs[i]["px"], gt[2], Ki, di, rv, tv)
        if got is not None:
            errs.append(float(np.linalg.norm(got[:2] - gt[:2])))
    errs = np.array(errs)

    json.dump({"K": K.tolist(), "dist": dist.tolist(),
               "rvec": rvec.tolist(), "tvec": tvec.tolist(),
               "rms_px": float(rms), "holdout_median_mm": float(np.median(errs)),
               "n": len(pairs)}, open(MODEL_JSON, "w"), indent=2)
    print(f"точек: {len(pairs)}   ошибка перепроекции: {rms:.2f} px")
    print(f"ОШИБКА НА ОТЛОЖЕННЫХ ТОЧКАХ (честная): медиана {np.median(errs):.1f} мм, "
          f"90-й перцентиль {np.percentile(errs, 90):.1f} мм")
    print(f"сохранено -> {MODEL_JSON}")


def _pixel_to_plane(px, plane_z, K, dist, rvec, tvec):
    """Cast the ray through pixel `px` and intersect it with the plane z = plane_z,
    in the arm's (command) coordinates. This is the whole point of the 3D model."""
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    t = np.asarray(tvec, dtype=np.float64).reshape(3)
    cam_pos = -R.T @ t                                   # camera centre, arm frame

    p = cv2.undistortPoints(np.array([[px]], dtype=np.float32),
                            np.asarray(K), np.asarray(dist))[0][0]
    dir_cam = np.array([p[0], p[1], 1.0])
    dir_world = R.T @ dir_cam                            # ray direction, arm frame
    if abs(dir_world[2]) < 1e-9:
        return None
    s = (plane_z - cam_pos[2]) / dir_world[2]
    if s <= 0:
        return None
    return cam_pos + s * dir_world


def pixel_to_floor(px, py, plane_z=FLOOR_Z):
    m = json.load(open(MODEL_JSON))
    p = _pixel_to_plane((float(px), float(py)), plane_z,
                        np.array(m["K"]), np.array(m["dist"]),
                        np.array(m["rvec"]), np.array(m["tvec"]))
    if p is None:
        return None
    return float(p[0]), float(p[1])


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "help"
    if cmd == "collect":
        collect()
    elif cmd == "fit":
        fit()
    elif cmd == "px":
        r = pixel_to_floor(float(sys.argv[2]), float(sys.argv[3]))
        print(f"пиксель -> пол: x={r[0]:.1f} y={r[1]:.1f} мм (радиус {math.hypot(*r):.1f})")
    else:
        print(__doc__)
        print("usage: calib3d.py collect | fit | px PX PY")
