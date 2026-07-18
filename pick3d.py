#!/usr/bin/env python3
"""Autonomous pick using the full 3D camera model (calib3d.py).

WHAT THIS FIXES vs pick.py
pick.py servoed the claw's markers onto the OBJECT'S pixel. That is wrong, and no
amount of tuning could save it: the claw markers and the object sit at different
heights, so making their projections coincide only puts them on the same viewing ray -
metres apart, in principle. In practice the claw would converge to 1-2 px of "perfect"
alignment and close on bare floor a couple of centimetres to the side, which is exactly
what the user kept seeing ("ты сильно правее сейчас от предмета").

With a real camera model the target pixel is COMPUTED, not assumed:
    object pixel --(ray x floor plane)--> object position in mm
    object position + grasp height       --(project)--> where the claw markers must appear
Servo to THAT pixel and the jaws close where the object actually is. Parallax is not
compensated - it is modelled away.

Everything is in COMMAND coordinates (see calib3d), so the floor is z = FLOOR_Z, not 0:
the arm sags, and commanding z=0 leaves the claw a couple of centimetres in the air.

VERIFYING A GRASP: not by the gripper reading. On a thin object it reads ~630 whether
it is holding or empty - indistinguishable from a closed-on-nothing 631. The only
honest check is to lift, rotate, and look at whether the object came along.
"""
import math
import subprocess
import sys
import time
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, "/home/astra/tools")
import calib          # jaw-marker detector
import calib3d        # 3D camera model
import kin
import pick           # object detector

ARM = "/home/astra/tools/arm"
OPEN, CLAMP = 156, 640
FLOOR_Z = calib3d.FLOOR_Z          # the floor, in command coordinates
GRASP_H = 12.0                     # how far above the floor the jaws should close
HOVER_H = 90.0

# Last good jaw-closing offset, in pixels. Measured live per grasp, but cached because
# the markers are occasionally hidden at certain poses - and the measurement is very
# consistent when it does succeed ((+7,+30), (+9,+30)).
_LAST_OFFSET = (8.0, 30.0)


def find_object():
    """Same blob detector as pick.py, minus its zone gate: that gate belonged to the old
    homography (valid only inside the swept plane). The 3D model has no such hull - the
    real limit is simply whether the arm can reach the point, which we check directly."""
    img = pick.frame()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, pick.OBJ_LO, pick.OBJ_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, _, stats, cent = cv2.connectedComponentsWithStats(m)
    best = [(stats[i, cv2.CC_STAT_AREA], i) for i in range(1, n)
            if pick.MIN_AREA <= stats[i, cv2.CC_STAT_AREA] <= pick.MAX_AREA]
    if not best:
        return None
    best.sort(key=lambda t: -t[0])
    i = best[0][1]

    # Refuse a blob that touches the frame border. We aim at the object's FLOOR CONTACT,
    # i.e. the bottom of its silhouette - but if the silhouette runs off the edge of the
    # image, that bottom is the edge of the picture, not the bottom of the object, and we
    # would confidently aim at a point that does not exist.
    x0, y0 = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
    w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
    H, W = m.shape[:2]
    if x0 <= 1 or y0 <= 1 or x0 + w >= W - 1 or y0 + h >= H - 1:
        print("предмет обрезан краем кадра — точка касания пола не видна, не берусь")
        return None

    return float(cent[i][0]), float(y0 + h - 2)


def project(xyz):
    """3D point (arm/command frame) -> pixel, via the calibrated camera."""
    import json
    m = json.load(open(calib3d.MODEL_JSON))
    p, _ = cv2.projectPoints(np.array([xyz], dtype=np.float32),
                             np.array(m["rvec"]), np.array(m["tvec"]),
                             np.array(m["K"]), np.array(m["dist"]))
    return tuple(float(v) for v in p.reshape(2))


def goto(x, y, z, ms=1200):
    sol = kin.ik_search(x, y, z, pitch_lo=calib.PITCH_LO, pitch_hi=calib.PITCH_HI,
                        prefer=(calib.PITCH_LO + calib.PITCH_HI) / 2)
    if not sol:
        return False
    calib3d.arm_step(",".join(f"{j}:{sol[j]}" for j in (6, 5, 4, 3)), ms)
    return True


def gripper():
    return int(subprocess.run([ARM, "get", "1"], capture_output=True, text=True).stdout.strip())


def measure_jaw_offset():
    """How far the jaws' midpoint MOVES when they close, in pixels.

    This is the bug that defeated every earlier attempt. The markers sit on the jaw
    tips, and the jaws swing on pivots - so the midpoint of the OPEN jaws is not where
    they will actually meet. Measured here: closing shifts the midpoint 25 px (dy=+25),
    which dwarfs both the parallax correction (~8 px) and the servo tolerance (2-3 px).
    Aiming the open jaws at the object therefore closed them a good centimetre past it,
    every single time, no matter how precisely the servo converged.

    Measure it at the current pose (it depends on the claw's orientation), so we can aim
    the OPEN jaws at target - offset and have them CLOSE on the target."""
    calib3d.arm_step(f"1:{OPEN}", 700)
    a = calib.find_claw()
    calib3d.arm_step(f"1:{CLAMP}", 700)
    b = calib.find_claw()
    calib3d.arm_step(f"1:{OPEN}", 700)
    if a is None or b is None:
        # Falling back to (0,0) would be the worst possible answer: a zero offset is
        # precisely the wrong assumption that made every early grasp miss. If the markers
        # cannot be seen, use the last good measurement - it is very stable across poses
        # (+7..+9, +30) - and say so, rather than quietly aiming at a known-bad target.
        if _LAST_OFFSET is not None:
            print(f"  (маркеры не видны — беру прошлое смещение {_LAST_OFFSET})")
            return _LAST_OFFSET
        return None
    off = (b[0] - a[0], b[1] - a[1])
    globals()["_LAST_OFFSET"] = off
    return off


def pick_object(iters=4, tol_px=3.0):
    px = find_object()
    if px is None:
        print("предмет не найден")
        return False

    obj = calib3d.pixel_to_floor(*px)          # ray x floor -> where it really is, in mm
    if obj is None:
        print("не могу спроецировать предмет на пол")
        return False
    ox, oy = obj
    z = FLOOR_Z + GRASP_H
    if not kin.reachable(ox, oy, z):
        print(f"предмет вне досягаемости (r={math.hypot(ox,oy):.0f} мм)")
        return False

    target_px = project([ox, oy, z])           # where the CLAW must appear, not the object
    print(f"предмет: пиксель ({px[0]:.0f},{px[1]:.0f}) -> x={ox:.0f} y={oy:.0f} мм "
          f"(r={math.hypot(ox,oy):.0f})")
    print(f"клешня должна встать в пиксель ({target_px[0]:.0f},{target_px[1]:.0f}) "
          f"— он не совпадает с пикселем предмета, это и есть параллакс")

    calib3d.arm_step(f"1:{OPEN}", 600)
    if not goto(ox, oy, FLOOR_Z + HOVER_H):
        print("не могу подойти")
        return False

    # Measure the jaw-closing shift up here, clear of the object - closing the jaws down
    # at the target would just shove it away.
    off = measure_jaw_offset()
    if off is None:
        print("не могу определить смещение губок — отказываюсь целиться вслепую")
        return False
    jdx, jdy = off
    aim_px = (target_px[0] - jdx, target_px[1] - jdy)
    print(f"смещение губок при смыкании: ({jdx:+.0f},{jdy:+.0f}) px -> "
          f"целюсь раскрытыми в ({aim_px[0]:.0f},{aim_px[1]:.0f}), чтобы СОМКНУЛИСЬ на предмете")

    if not goto(ox, oy, z):
        print("не могу опуститься")
        return False

    x, y = ox, oy
    for i in range(iters):
        claw = calib.find_claw()
        if claw is None:
            # Don't abort: the 3D model already put us within ~5 mm. Closing open-loop is
            # a far better bet than giving up, and beats aiming at a guess.
            print("  маркеры не видны — доверяюсь модели, без коррекции")
            break
        err = math.dist(claw, aim_px)
        print(f"  итерация {i}: клешня=({claw[0]:.0f},{claw[1]:.0f}) "
              f"цель=({aim_px[0]:.0f},{aim_px[1]:.0f}) ошибка={err:.1f} px")
        if err <= tol_px:
            break
        # convert the pixel error to millimetres on the plane the claw is actually on
        here = calib3d.pixel_to_floor(*claw, plane_z=z)
        want = calib3d.pixel_to_floor(*aim_px, plane_z=z)
        if here is None or want is None:
            break
        x += want[0] - here[0]
        y += want[1] - here[1]
        if not kin.reachable(x, y, z) or not goto(x, y, z, 900):
            print("  коррекция недостижима")
            return False

    img = pick.frame()
    cv2.drawMarker(img, (int(target_px[0]), int(target_px[1])), (0, 255, 255),
                   cv2.MARKER_CROSS, 18, 2)
    cv2.drawMarker(img, (int(px[0]), int(px[1])), (255, 0, 255),
                   cv2.MARKER_TILTED_CROSS, 18, 2)
    cv2.imwrite("/home/astra/tools/calib/pre_grasp3d.jpg", img)

    calib3d.arm_step(f"1:{CLAMP}", 900)
    print(f"  клешня сомкнута: {gripper()}  (число НЕ доказывает захват — проверяем глазами)")
    goto(x, y, FLOOR_Z + HOVER_H, 1400)        # lift, jaws stay closed
    return True


def held(orig_px, tol=25.0):
    """Did we actually pick it up? The gripper reading cannot tell us - on a thin object
    it reads ~618-631 whether it is holding or empty. So look instead: with the arm
    lifted, is the object still lying where it was? If it has vanished from that spot, it
    came up with the claw."""
    still = find_object()
    if still is None:
        return True                       # nothing on the floor any more -> we have it
    return math.dist(still, orig_px) > tol


def place(x, y, ms=1300):
    """Set the object down at (x, y) and back off, leaving the jaws open."""
    z = FLOOR_Z + GRASP_H
    if not goto(x, y, FLOOR_Z + HOVER_H, ms) or not goto(x, y, z, ms):
        return False
    calib3d.arm_step(f"1:{OPEN}", 700)
    goto(x, y, FLOOR_Z + HOVER_H, ms)
    return True


def train(cycles=5):
    """Practise: grasp, carry, set down somewhere else, repeat - reporting an honest
    success rate. Each attempt is verified by LOOKING (see `held`), never by the gripper
    number."""
    # Spots chosen so the object lands well INSIDE the frame. An earlier set drifted it to
    # the bottom edge, where its blob is clipped and the floor-contact point we aim at is
    # simply wrong.
    spots = [(150.0, 60.0), (175.0, 100.0), (140.0, 120.0), (130.0, 80.0), (150.0, 110.0)]
    wins = 0
    for i in range(cycles):
        print(f"\n===== попытка {i+1}/{cycles} =====")
        subprocess.run([ARM, "home"], stdout=subprocess.DEVNULL)
        time.sleep(1.0)
        before = find_object()
        if before is None:
            print("предмет не найден — прекращаю")
            break
        if not pick_object():
            print("  ЗАХВАТ НЕ ВЫПОЛНЕН")
            continue
        ok = held(before)
        wins += ok
        print(f"  РЕЗУЛЬТАТ: {'ВЗЯЛ' if ok else 'МИМО'}  (успехов {wins}/{i+1})")
        if ok:
            tx, ty = spots[i % len(spots)]
            if kin.reachable(tx, ty, FLOOR_Z + GRASP_H):
                place(tx, ty)
                print(f"  положил в ({tx:.0f}, {ty:.0f}) мм")
            else:
                calib3d.arm_step(f"1:{OPEN}", 700)
        else:
            calib3d.arm_step(f"1:{OPEN}", 700)
    print(f"\n===== ИТОГ: {wins} из {cycles} =====")
    subprocess.run([ARM, "home"], stdout=subprocess.DEVNULL)
    return wins


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "train":
        train(int(sys.argv[2]) if len(sys.argv) > 2 else 5)
    else:
        pick_object()
