#!/usr/bin/env python3
"""Turn-arc calibration probe.

One steered pulse -> measure achieved yaw from the wrist camera (forward-facing)
via ORB + essential-matrix recoverPose. No encoders, so the camera IS the sensor.

Usage: turncal.py <left|right> <steer_angle> <speed> <duration_s> <outdir> <tag>
Prints:  inliers, measured yaw (deg). Saves A/B frames as <tag>_A.jpg/<tag>_B.jpg.

Neck (arm servo 6) is assumed already aimed forward (pos 500). We do NOT touch the
arm here so A and B share the exact same camera->body transform; only the body turns.
"""
import sys, os, time, math
sys.path.insert(0, "/home/astra/tools")
import car
import cv2
import numpy as np

FOV_DEG = 60.0  # uncalibrated assumption, same as nav.py


def intr(w, h, fov=FOV_DEG):
    fx = (w / 2.0) / math.tan(math.radians(fov) / 2.0)
    return np.array([[fx, 0, w / 2.0], [0, fx, h / 2.0], [0, 0, 1.0]])


def measure_yaw(fa, fb):
    ga = cv2.imread(fa, cv2.IMREAD_GRAYSCALE)
    gb = cv2.imread(fb, cv2.IMREAD_GRAYSCALE)
    orb = cv2.ORB_create(1500)
    ka, da = orb.detectAndCompute(ga, None)
    kb, db = orb.detectAndCompute(gb, None)
    if da is None or db is None:
        return None, 0, 0, 0
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = [p for p in bf.knnMatch(da, db, k=2) if len(p) == 2]
    good = [m for m, n in pairs if m.distance < 0.75 * n.distance]
    if len(good) < 8:
        return None, len(ka), len(kb), len(good)
    ptsA = np.float32([ka[m.queryIdx].pt for m in good])
    ptsB = np.float32([kb[m.trainIdx].pt for m in good])
    K = intr(ga.shape[1], ga.shape[0])
    E, mask = cv2.findEssentialMat(ptsA, ptsB, K, cv2.RANSAC, 0.999, 1.0)
    if E is None or E.shape != (3, 3):
        return None, len(ka), len(kb), len(good)
    inl = int(mask.sum()) if mask is not None else 0
    _, R, t, _ = cv2.recoverPose(E, ptsA, ptsB, K, mask=mask)
    # yaw about camera vertical (down) axis -> robot heading change (nav.py convention)
    dtheta = -math.degrees(math.atan2(R[0, 2], R[2, 2]))
    tdir = t.ravel()
    return dtheta, len(ka), len(kb), inl, tdir


def main():
    side, ang, spd, dur, outdir, tag = (
        sys.argv[1], int(sys.argv[2]), int(sys.argv[3]),
        float(sys.argv[4]), sys.argv[5], sys.argv[6])
    fa = os.path.join(outdir, f"{tag}_A.jpg")
    fb = os.path.join(outdir, f"{tag}_B.jpg")

    time.sleep(0.3)
    car.snapshot(fa)
    time.sleep(0.2)
    car.steer(side, ang)
    time.sleep(0.25)
    car.move("forward", spd, dur)
    car.steer("center", 90)
    time.sleep(0.5)
    car.snapshot(fb)

    res = measure_yaw(fa, fb)
    if res[0] is None:
        print(f"MEASURE FAIL: kpA={res[1]} kpB={res[2]} good={res[3]}")
        return
    dtheta, kpa, kpb, inl, tdir = res
    print(f"cmd: steer {side} {ang}, forward speed {spd} {dur}s")
    print(f"kpA={kpa} kpB={kpb} inliers={inl}")
    print(f"MEASURED YAW = {dtheta:+.1f} deg (recoverPose)")
    print(f"unit_t (cam frame x,y,z) = [{tdir[0]:+.2f} {tdir[1]:+.2f} {tdir[2]:+.2f}]")


if __name__ == "__main__":
    main()
