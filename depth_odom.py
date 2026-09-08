#!/usr/bin/env python3
"""depth_odom.py - yaw from the Aurora930 depth map, by matching one frame's
range profile against another's.

Why this exists: the chassis has no wheel encoders and no IMU (the Aurora930
streams none either), `nav.py turn` carries an unresolved sign/scale bug, and the
ORB yaw estimate goes quietly to ~0 on a low-texture scene like a plain wall or a
curtain. Depth does not care about texture, so it is the one remaining route to a
heading that is measured rather than assumed.

The method is 1D scan matching, not full ICP: collapse the depth image to a
"range per column" profile - a virtual laser scan across the camera's 74.8 deg
horizontal field - then find the horizontal shift that best aligns two profiles.
Cheap enough for the Pi, and robust because it throws away everything except the
shape of the range-vs-bearing curve.

A pixel shift becomes an angle through the real intrinsics, which
aurora_streamer publishes in meta.json (fx=418.33 at 640x400 -> 0.137 deg/px).

WHAT IS ACTUALLY VALIDATED (2026-09-09), and what is not:

  * Static case is clean: with nothing moving it reads 0.07 deg, score 4 mm,
    margin 49 mm. It does not invent motion.
  * Sign is correct and the response is monotonic over +-10 deg.
  * Scale under-reads: slope est/true = 0.70 against the arm turntable.
  * Best pose is `deck` (wrist 682). Raising the wrist makes it worse, not
    better - 720 gave slope 0.67, and 750 gave 0.44 with margins collapsing to
    6 mm, i.e. no lock at all. More of the frame returning depth beats more of
    the frame containing vertical structure.

  The 0.70 is NOT established as the chassis scale. It was measured by rotating
  the ARM base, and the camera sits off that rotation axis, so every rotation is
  also a small arc translation - a bias this test cannot separate from a genuine
  scale error. Calibrating against a real chassis turn needs a working forward
  range sense for the preflight, and the ultrasonic is frozen (see safety.py
  is_sonic_stuck). Until then: trust the SIGN, treat the MAGNITUDE as a lower
  bound, and check `margin` before believing anything.

  `margin` is the confidence signal that matters. Every estimate with a margin
  under ~20 mm in testing was garbage; the good ones sat at 39-57 mm.

CLI:
  depth_odom.py profile              print the current range profile
  depth_odom.py yaw [--wait S]       yaw between now and S seconds later
  depth_odom.py calib --units U,...  rotate the ARM base by U units and compare
                                     the estimate against that known angle
"""
import argparse
import json
import math
import subprocess
import sys
import time
import urllib.request

import depth as depth_mod

META_URL = "http://127.0.0.1:8090/meta"
VENV_PY = "/home/astra/tools/venv/bin/python3"
ARM_PY = "/home/astra/robotics/arm.py"
UNITS_PER_DEG = 4.0          # kin.py: servo units per degree, base servo 6
FX_FALLBACK = 418.33

# Columns are pooled into bins before matching: a single column is mostly noise
# and holes, a bin of 4 is a stable range sample and makes the search 4x cheaper.
BIN = 4
# A profile bin needs this fraction of its pixels to carry a return, or it counts
# as a hole and takes no part in the match.
MIN_BIN_FILL = 0.15
# Below this the cost curve is flat: the scene had no structure to lock onto and
# the "best" shift is noise. Measured - good matches ran 39-57 mm, and every
# estimate under 20 mm was wrong by degrees.
MIN_MARGIN_MM = 20.0


def intrinsics(url=META_URL, timeout=4):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            m = json.loads(r.read())
        return float(m.get("fx") or FX_FALLBACK), m
    except Exception:
        return FX_FALLBACK, {}


def profile(w, h, a, band=(0.35, 0.78), binw=BIN):
    """Virtual laser scan: median non-zero depth (mm) per column-bin, or None.

    The band excludes the top of the frame (ceiling/backlight, where structured
    light gives nothing) and the bottom (the robot's own gripper, which would
    otherwise anchor the match to a part of the scene that never moves).
    """
    y0, y1 = int(h * band[0]), int(h * band[1])
    nbins = w // binw
    out = []
    for b in range(nbins):
        x0, x1 = b * binw, (b + 1) * binw
        vals = []
        for y in range(y0, y1):
            row = y * w
            for x in range(x0, x1):
                v = a[row + x]
                if v:
                    vals.append(v)
        need = MIN_BIN_FILL * (y1 - y0) * binw
        if len(vals) >= need and vals:
            vals.sort()
            out.append(float(vals[len(vals) // 2]))
        else:
            out.append(None)
    return out


def _sad(p, q, shift):
    """Mean |difference| over bins valid in both after shifting q by `shift`."""
    tot, n = 0.0, 0
    for i, pv in enumerate(p):
        j = i + shift
        if pv is None or j < 0 or j >= len(q):
            continue
        qv = q[j]
        if qv is None:
            continue
        tot += abs(pv - qv)
        n += 1
    return (tot / n, n) if n else (float("inf"), 0)


def match(p, q, max_shift_bins=40, min_overlap=20):
    """Best bin shift aligning p onto q, with a parabolic sub-bin refinement.

    Returns (shift_bins_float, score_mm, overlap, margin). `margin` is how much
    worse the second-best distinct shift is - a flat cost curve (small margin)
    means the scene had no structure to lock onto and the answer is not usable.
    """
    scores = {}
    for s in range(-max_shift_bins, max_shift_bins + 1):
        sc, n = _sad(p, q, s)
        if n >= min_overlap:
            scores[s] = sc
    if not scores:
        return None, None, 0, None
    best = min(scores, key=scores.get)
    best_sc = scores[best]
    _, overlap = _sad(p, q, best)

    # margin against the best shift at least 3 bins away, so the neighbouring
    # bins of the same minimum do not count as a rival
    rivals = [v for s, v in scores.items() if abs(s - best) >= 3]
    margin = (min(rivals) - best_sc) if rivals else None

    sub = float(best)
    if best - 1 in scores and best + 1 in scores:
        y0, y1, y2 = scores[best - 1], best_sc, scores[best + 1]
        den = y0 - 2 * y1 + y2
        if abs(den) > 1e-9:
            sub = best + 0.5 * (y0 - y2) / den
    return sub, best_sc, overlap, margin


def shift_to_deg(shift_bins, fx, binw=BIN):
    """Pixel shift -> degrees, through the real focal length."""
    return math.degrees(math.atan((shift_bins * binw) / fx))


def grab():
    w, h, a = depth_mod.fetch()
    return w, h, a


def yaw_between(p0, p1, fx, require_margin=True, **kw):
    """(yaw_deg | None, info). None whenever the match is not trustworthy - an
    unusable estimate must not be indistinguishable from a confident zero.
    """
    shift, score, overlap, margin = match(p0, p1, **kw)
    if shift is None:
        return None, {"why": "no overlap between profiles"}
    info = {"shift_bins": round(shift, 2), "score_mm": round(score, 1),
            "overlap_bins": overlap,
            "margin_mm": None if margin is None else round(margin, 1)}
    if require_margin and (margin is None or margin < MIN_MARGIN_MM):
        info["why"] = (f"margin {info['margin_mm']} mm below {MIN_MARGIN_MM} mm - "
                       "the scene has no structure to lock onto")
        return None, info
    return shift_to_deg(shift, fx), info


def _arm(*args):
    return subprocess.run(["sudo", VENV_PY, ARM_PY, *[str(x) for x in args]],
                          capture_output=True, text=True, timeout=60)


def cmd_calib(units_list, settle=1.6):
    """Rotate the ARM base by a known angle and see what the estimator says.

    The camera is on the wrist, so servo 6 is a calibrated turntable for it - a
    way to test yaw estimation to a fraction of a degree without driving the
    chassis at all. It is not a perfect chassis analogue: the camera sits off the
    base axis, so a rotation also swings it through a small arc (about 9 mm at
    5 deg with the arm folded, against a scene 0.6-1.1 m away). Small angles keep
    that translation negligible; large ones do not.
    """
    fx, meta = intrinsics()
    print(f"fx={fx:.2f}  ({math.degrees(math.atan(320/fx))*2:.1f} deg horizontal FOV)")
    base0 = int(_arm("get", 6).stdout.strip())
    print(f"base servo 6 at {base0}; UNITS_PER_DEG={UNITS_PER_DEG}\n")
    print(f"{'units':>6} {'true deg':>9} {'est deg':>9} {'err':>7} "
          f"{'score':>7} {'margin':>7} {'ovl':>5}")
    rows = []
    try:
        for u in units_list:
            _arm("move", 6, base0)
            time.sleep(settle)
            w, h, a0 = grab()
            p0 = profile(w, h, a0)

            _arm("move", 6, base0 + u)
            time.sleep(settle)
            w, h, a1 = grab()
            p1 = profile(w, h, a1)

            true_deg = u / UNITS_PER_DEG
            # calibration wants to see the low-margin rows too, so it can learn
            # where the threshold belongs rather than being protected from them
            est, info = yaw_between(p0, p1, fx, require_margin=False)
            if est is None:
                print(f"{u:>6} {true_deg:>9.2f} {'--':>9} {'--':>7}  {info['why']}")
                continue
            err = est - true_deg
            print(f"{u:>6} {true_deg:>9.2f} {est:>9.2f} {err:>7.2f} "
                  f"{info['score_mm']:>7.0f} "
                  f"{str(info['margin_mm']):>7} {info['overlap_bins']:>5}")
            rows.append((true_deg, est))
    finally:
        _arm("move", 6, base0)
    if len(rows) >= 2:
        # least-squares slope through the origin: est = k * true
        num = sum(t * e for t, e in rows)
        den = sum(t * t for t, e in rows)
        if den:
            k = num / den
            print(f"\nslope est/true = {k:+.3f}  "
                  f"({'sign OK' if k > 0 else 'SIGN INVERTED'}, "
                  f"{'scale OK' if 0.8 < abs(k) < 1.2 else 'SCALE OFF'})")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("profile")
    y = sub.add_parser("yaw")
    y.add_argument("--wait", type=float, default=2.0)
    c = sub.add_parser("calib")
    c.add_argument("--units", default="-60,-40,-20,20,40,60")
    c.add_argument("--settle", type=float, default=1.6)
    args = ap.parse_args()

    if args.cmd == "profile":
        fx, _ = intrinsics()
        w, h, a = grab()
        p = profile(w, h, a)
        valid = [v for v in p if v is not None]
        print(f"{len(p)} bins of {BIN}px, {len(valid)} valid "
              f"({100*len(valid)/len(p):.0f}%), fx={fx:.1f}")
        for i in range(0, len(p), 8):
            chunk = p[i:i + 8]
            print("  " + " ".join("  --  " if v is None else f"{v/1000:5.2f}m"
                                  for v in chunk))
        return 0

    if args.cmd == "yaw":
        fx, _ = intrinsics()
        w, h, a0 = grab()
        p0 = profile(w, h, a0)
        time.sleep(args.wait)
        w, h, a1 = grab()
        p1 = profile(w, h, a1)
        est, info = yaw_between(p0, p1, fx)
        print(json.dumps({"yaw_deg": None if est is None else round(est, 2), **info}))
        return 0

    if args.cmd == "calib":
        return cmd_calib([int(u) for u in args.units.split(",")], args.settle)
    return 2


if __name__ == "__main__":
    sys.exit(main())
