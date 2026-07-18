#!/usr/bin/env python3
"""K-turn (3-point turn) rotation test for a weak-steering car.

forward+steer_right then backward+steer_left both rotate the body CW while
net translation ~cancels -> rotate (nearly) in place. Snap REF before and
AFTER, report yaw via central-disparity (dxyaw).
"""
import sys, os, time, math
sys.path.insert(0, "/home/astra/tools")
import car
import cv2
import numpy as np

FOV = 60.0


def yaw(fa, fb, band=90):
    ga = cv2.imread(fa, cv2.IMREAD_GRAYSCALE); gb = cv2.imread(fb, cv2.IMREAD_GRAYSCALE)
    h, w = ga.shape
    fx = (w / 2.0) / math.tan(math.radians(FOV) / 2.0); cx = w / 2.0
    orb = cv2.ORB_create(2000)
    ka, da = orb.detectAndCompute(ga, None); kb, db = orb.detectAndCompute(gb, None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = [p for p in bf.knnMatch(da, db, k=2) if len(p) == 2]
    good = [m for m, n in pairs if m.distance < 0.75 * n.distance]
    dxc = [kb[m.trainIdx].pt[0] - ka[m.queryIdx].pt[0] for m in good
           if abs(ka[m.queryIdx].pt[0] - cx) < band]
    dall = [kb[m.trainIdx].pt[0] - ka[m.queryIdx].pt[0] for m in good]
    if not dxc:
        dxc = dall
    return float(np.median(dxc)) / fx * 180 / math.pi, len(good), len(dxc)


def main():
    side = sys.argv[1]              # "right" -> CW ; forward turns toward this side
    ang = int(sys.argv[2]); spd = int(sys.argv[3]); dur = float(sys.argv[4])
    outdir = sys.argv[5]; tag = sys.argv[6]
    other = "left" if side == "right" else "right"
    ref = os.path.join(outdir, f"{tag}_REF.jpg")
    aft = os.path.join(outdir, f"{tag}_AFT.jpg")
    mid = os.path.join(outdir, f"{tag}_MID.jpg")

    time.sleep(0.3); car.snapshot(ref)
    # phase 1: forward + steer toward `side`
    car.steer(side, ang); time.sleep(0.25)
    car.move("forward", spd, dur)
    car.steer("center", 90); time.sleep(0.4)
    car.snapshot(mid)
    # phase 2: backward + steer the OTHER way (rotates the SAME rotational sense)
    car.steer(other, ang); time.sleep(0.25)
    car.move("backward", spd, dur)
    car.steer("center", 90); time.sleep(0.5)
    car.snapshot(aft)

    y1, n1, c1 = yaw(ref, mid)
    y2, n2, c2 = yaw(ref, aft)
    print(f"K-turn side={side} ang={ang} spd={spd} dur={dur}s")
    print(f"  after FWD phase : yaw={y1:+.1f} deg (matches {n1}, centre {c1})")
    print(f"  after FULL Kturn: yaw={y2:+.1f} deg (matches {n2}, centre {c2})")
    print("  (+ = LEFT/CCW, - = RIGHT/CW)")


if __name__ == "__main__":
    main()
