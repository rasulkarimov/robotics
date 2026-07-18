#!/usr/bin/env python3
"""In-code clearance / obstacle metric from a single wrist-camera frame.

Motivation (2026-07-18, "1+2 делай"): pull perception INTO the robot loop so a
batched maneuver script can gate its own motion, instead of round-tripping every
frame through the LLM. No ultrasonic on this car (dead) -> the camera IS the sensor.

Idea: this room's real obstacles (sofa=blue, wardrobe/door=brown, bed) are all
COLOURFUL or DARK, sitting on a light near-neutral TILE floor (light walls are the
far boundary). So "free floor ahead" = how far UP the frame the light, low-saturation,
low-texture floor extends in a column before a dark/colourful/ textured region intrudes.
Report that as a fraction per third (left/centre/right); 0 = obstacle at the bumper,
~1 = open floor to the horizon. The two red gripper fingers live in the bottom corners
of every wrist-cam frame and are masked out (they are bright red = high saturation).

Works on the LEVEL forward frame (servo5~514, servo6=500) so one snapshot yields both
yaw (via dxyaw) and clearance without a camera tilt. A steeper down-tilt gives a nearer,
more sensitive read when the level metric is ambiguous.

CLI:
  perceive.py clear IMG [IMG ...]         # print metrics per image
  perceive.py montage OUT.png IMG [IMG..] # labelled montage w/ overlaid boundary
"""
import sys
import math
import cv2
import numpy as np

# Column bands (fractions of width). Corners avoided -> gripper fingers excluded.
BANDS = {"left": (0.12, 0.37), "center": (0.38, 0.62), "right": (0.63, 0.88)}
# A column is "floor" from the bottom up until a sustained non-floor run begins.
RUN = 6            # rows of sustained non-floor to call it an obstacle boundary
IGNORE_BOTTOM = 0.06   # skip the very bottom rows (car body / motion blur edge)


def _floor_masks(bgr):
    """Return (is_floor, skip, dbg).
      is_floor: light, low-saturation floor/wall (NOT a near furniture hazard)
      skip:     gripper-finger assembly (red tops + black bases) -> ignore, not obstacle
    Furniture in this room is brown/blue/dark on a neutral tile floor + light walls, so
    dark|saturated flags the real hazards; texture is dropped (tile grout lines tripped it)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].astype(np.int16), hsv[..., 1], hsv[..., 2]
    h, w = V.shape

    # Gripper fingers: bright red tops. Dilate to swallow the adjacent black base + arm,
    # so the whole finger assembly is SKIPPED (occlusion), never counted as an obstacle.
    red = ((S > 80) & ((H < 12) | (H > 168))).astype(np.uint8)
    skip = cv2.dilate(red, np.ones((41, 41), np.uint8), iterations=1) > 0

    # Robust floor reference: sample the bottom 40%, but let ONLY near-neutral (low-sat)
    # non-skip pixels vote. A coloured obstacle intruding into the bottom can't poison the
    # reference (that was the failure mode of a single fixed patch).
    band = np.zeros_like(S, dtype=bool)
    band[int(h * 0.60):int(h * 0.96), :] = True
    neutral = band & (~skip) & (S < 45)
    if int(neutral.sum()) > 300:
        refV = float(np.median(V[neutral]))
        refS = float(np.median(S[neutral]))
    else:  # almost nothing neutral low -> obstacle fills the near field
        refV, refS = float(np.median(V[band & ~skip])), 0.0

    # RELATIVE to the floor reference: an obstacle is notably darker OR more colourful
    # than the tile right in front. Catches the smooth brown door (more chroma than grey
    # tile) and the wood wardrobe alike, without absolute tuning. Light walls (~= floor
    # in S and V) stay "floor-like" -> a known blind spot for a bare pale wall dead ahead.
    dark = V < (refV - 42)                       # darker than the floor
    colourful = S.astype(np.int16) > (refS + 22)  # more saturated than the floor
    not_floor = (dark | colourful) & (~skip)
    is_floor = ~not_floor
    dbg = {"refV": refV, "refS": refS}
    return is_floor, skip, dbg


def clearance(path):
    bgr = cv2.imread(path)
    if bgr is None:
        raise FileNotFoundError(path)
    h, w = bgr.shape[:2]
    is_floor, skip, dbg = _floor_masks(bgr)
    bottom_ignore = int(h * (1 - IGNORE_BOTTOM))

    out = {"file": path.split("/")[-1], "w": w, "h": h, "bands": {}, **dbg}
    boundaries = {}
    for name, (f0, f1) in BANDS.items():
        c0, c1 = int(w * f0), int(w * f1)
        tops = []
        for c in range(c0, c1, 2):
            col = is_floor[:, c]
            sk = skip[:, c]
            top = 0  # row index where floor stops (obstacle boundary); 0 => floor to top
            run = 0
            # scan upward from just above the ignored bottom
            for r in range(bottom_ignore - 1, -1, -1):
                if sk[r]:            # gripper finger / arm: occlusion, not an obstacle
                    run = 0
                    continue
                if not col[r]:
                    run += 1
                    if run >= RUN:
                        top = r + RUN  # boundary at start of the sustained non-floor run
                        break
                else:
                    run = 0
            tops.append(top)
        # fraction of frame height that is free floor (higher = more open).
        # boundary row 'top' near bottom => small free fraction.
        med_top = float(np.median(tops))
        free_frac = round(1.0 - med_top / h, 3)
        out["bands"][name] = free_frac
        boundaries[name] = med_top
    out["min"] = round(min(out["bands"].values()), 3)
    # recommended clearer side
    out["clearest"] = max(out["bands"], key=out["bands"].get)
    out["_boundaries"] = boundaries
    return out


def montage(paths, outpath):
    tiles = []
    for p in paths:
        bgr = cv2.imread(p)
        if bgr is None:
            continue
        m = clearance(p)
        h, w = bgr.shape[:2]
        # draw band boundaries
        for name, (f0, f1) in BANDS.items():
            c0, c1 = int(w * f0), int(w * f1)
            row = int(m["_boundaries"][name])
            col = (0, 255, 0) if m["bands"][name] > 0.45 else (
                0, 165, 255) if m["bands"][name] > 0.30 else (0, 0, 255)
            cv2.line(bgr, (c0, row), (c1, row), col, 2)
            cv2.putText(bgr, f"{m['bands'][name]:.2f}", (c0 + 2, max(row - 4, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        label = f"{m['file']} min={m['min']:.2f} clr={m['clearest']}"
        cv2.rectangle(bgr, (0, 0), (w, 18), (0, 0, 0), -1)
        cv2.putText(bgr, label, (3, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(bgr)
    if not tiles:
        raise SystemExit("no images")
    hmin = min(t.shape[0] for t in tiles)
    tiles = [cv2.resize(t, (int(t.shape[1] * hmin / t.shape[0]), hmin)) for t in tiles]
    grid = cv2.hconcat(tiles)
    cv2.imwrite(outpath, grid)
    return outpath


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    cmd = sys.argv[1]
    if cmd == "clear":
        for p in sys.argv[2:]:
            m = clearance(p)
            b = m["bands"]
            print(f"{m['file']:22s} L={b['left']:.2f} C={b['center']:.2f} "
                  f"R={b['right']:.2f}  min={m['min']:.2f} clearest={m['clearest']} "
                  f"(refV={m['refV']:.0f})")
    elif cmd == "montage":
        out = montage(sys.argv[3:], sys.argv[2])
        print(f"montage -> {out}")
    else:
        print(f"unknown cmd {cmd}")


if __name__ == "__main__":
    main()
