#!/usr/bin/env python3
"""Robust yaw from central-feature horizontal disparity.

For a forward-facing camera, a matched feature near the image centre (phi~0)
shifts horizontally by ~ -yaw regardless of forward translation (the
translation term scales with phi, ->0 at centre). So the median dx of
centre-band matches is a clean yaw estimate. Sign: scene shifts RIGHT (+dx)
=> camera/body turned LEFT (CCW, +yaw). yaw_deg = median_dx / fx * 57.3.
"""
import sys, math
import cv2
import numpy as np

FOV_DEG = 60.0


def analyze(fa, fb, band=70):
    ga = cv2.imread(fa, cv2.IMREAD_GRAYSCALE)
    gb = cv2.imread(fb, cv2.IMREAD_GRAYSCALE)
    h, w = ga.shape
    fx = (w / 2.0) / math.tan(math.radians(FOV_DEG) / 2.0)
    cx = w / 2.0
    orb = cv2.ORB_create(2000)
    ka, da = orb.detectAndCompute(ga, None)
    kb, db = orb.detectAndCompute(gb, None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = [p for p in bf.knnMatch(da, db, k=2) if len(p) == 2]
    good = [m for m, n in pairs if m.distance < 0.75 * n.distance]
    dxs_all, dxs_ctr = [], []
    for m in good:
        pa = ka[m.queryIdx].pt
        pb = kb[m.trainIdx].pt
        dx = pb[0] - pa[0]
        dxs_all.append(dx)
        if abs(pa[0] - cx) < band and abs(pb[0] - cx) < band:
            dxs_ctr.append(dx)
    if not dxs_ctr:
        dxs_ctr = dxs_all
    med_ctr = float(np.median(dxs_ctr))
    med_all = float(np.median(dxs_all))
    yaw_ctr = med_ctr / fx * 180 / math.pi
    yaw_all = med_all / fx * 180 / math.pi
    print(f"{fa.split('/')[-1]} -> {fb.split('/')[-1]}: matches={len(good)} "
          f"centre={len(dxs_ctr)}")
    print(f"  median dx  centre={med_ctr:+.1f}px  all={med_all:+.1f}px  (fx={fx:.0f})")
    print(f"  YAW centre={yaw_ctr:+.2f} deg   all={yaw_all:+.2f} deg   "
          f"(+ = turned LEFT/CCW)")
    return yaw_ctr


if __name__ == "__main__":
    analyze(sys.argv[1], sys.argv[2])
