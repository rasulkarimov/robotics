#!/usr/bin/env python3
"""depth.py - read the Aurora930 depth map (served by aurora-camera.service on
:8090 /depth) and turn it into the numbers you need to orient: how far the
nearest thing is, and whether the left / centre / right of the view is clear.

The camera is on the ARM WRIST, so this measures whatever the arm is pointing at.
Aim first (`nav.py lookout --view horizon`, or an arm pose) then read.

Depth is uint16 millimetres, row-major, 0 = no return (too close <~150mm, too far
>~4m, or a non-reflecting surface). All stats ignore zeros.

CLI:
  depth.py                 readout + coarse ASCII map
  depth.py ranges [--json] left/centre/right/min distances
  depth.py clear [MM]      exit 0 if >= MM (default 400) of free space ahead, else 1
"""
import argparse
import array
import json
import sys
import urllib.request

URL = "http://127.0.0.1:8090/depth"
NODATA = 0


def fetch(url=URL, timeout=4):
    """-> (width, height, array('H') of length w*h)."""
    with urllib.request.urlopen(url, timeout=timeout) as r:
        w = int(r.headers.get("X-Width", 0))
        h = int(r.headers.get("X-Height", 0))
        raw = r.read()
    a = array.array("H")
    a.frombytes(raw)
    if not w or not h:
        # fall back: assume 640 wide
        w = 640
        h = len(a) // w
    return w, h, a


def _pct(values, p):
    """p-th percentile of a list (0..100); None if empty."""
    if not values:
        return None
    s = sorted(values)
    k = max(0, min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


def sectors(w, h, a, band=(0.30, 0.80), near_pct=5):
    """Split a central horizontal band into left/centre/right thirds.
    Each value is the near_pct-th percentile of non-zero depths in that third -
    a noise-robust 'nearest surface' for that direction. Also returns the single
    centre pixel and the overall nearest point in the band.
    """
    y0, y1 = int(h * band[0]), int(h * band[1])
    thirds = {"left": (0, w // 3), "center": (w // 3, 2 * w // 3), "right": (2 * w // 3, w)}
    out = {}
    band_vals = []
    for name, (x0, x1) in thirds.items():
        vals = []
        for y in range(y0, y1):
            row = y * w
            for x in range(x0, x1):
                v = a[row + x]
                if v != NODATA:
                    vals.append(v)
        out[name] = _pct(vals, near_pct)
        band_vals += vals
    out["min"] = min(band_vals) if band_vals else None
    out["nearp"] = _pct(band_vals, near_pct)
    out["center_point"] = a[(h // 2) * w + w // 2] or None
    cov = len(band_vals) / max(1, (y1 - y0) * w)
    out["coverage"] = round(cov, 3)
    return out


def ascii_map(w, h, a, cols=36, rows=12, far=4000):
    """Coarse depth grid: '#' near, '.' far, ' ' no data. Nearest wins per cell."""
    glyphs = "@%#*+=-:. "
    lines = []
    for gy in range(rows):
        y0, y1 = gy * h // rows, (gy + 1) * h // rows
        line = []
        for gx in range(cols):
            x0, x1 = gx * w // cols, (gx + 1) * w // cols
            near = None
            for y in range(y0, y1, 3):
                row = y * w
                for x in range(x0, x1, 3):
                    v = a[row + x]
                    if v and (near is None or v < near):
                        near = v
            if near is None:
                line.append(" ")
            else:
                idx = min(len(glyphs) - 1, int(near / far * (len(glyphs) - 1)))
                line.append(glyphs[idx])
        lines.append("".join(line))
    return "\n".join(lines)


def _fmt(mm):
    return "  --  " if mm is None else f"{mm/1000:5.2f}m"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("ranges").add_argument("--json", action="store_true")
    c = sub.add_parser("clear")
    c.add_argument("mm", nargs="?", type=int, default=400)
    ap.add_argument("--url", default=URL)
    args = ap.parse_args(argv)

    try:
        w, h, a = fetch(args.url)
    except Exception as e:
        print(f"depth unavailable: {e}", file=sys.stderr)
        return 3
    s = sectors(w, h, a)

    if args.cmd == "clear":
        ahead = s["center"] if s["center"] is not None else s["nearp"]
        if ahead is None:
            print("unknown (no depth data ahead)", file=sys.stderr)
            return 2
        ok = ahead >= args.mm
        print(f"{'CLEAR' if ok else 'BLOCKED'} ahead={ahead/1000:.2f}m need={args.mm/1000:.2f}m")
        return 0 if ok else 1

    if args.cmd == "ranges":
        if args.json:
            print(json.dumps(s))
        else:
            print(f"left {_fmt(s['left'])}   center {_fmt(s['center'])}   right {_fmt(s['right'])}")
            print(f"nearest in view {_fmt(s['min'])}   center pixel {_fmt(s['center_point'])}"
                  f"   coverage {s['coverage']*100:.0f}%")
        return 0

    # default: full readout
    print(f"Aurora depth {w}x{h}, coverage {s['coverage']*100:.0f}% (0=no return)")
    print(f"  left   {_fmt(s['left'])}")
    print(f"  center {_fmt(s['center'])}   (centre pixel {_fmt(s['center_point'])})")
    print(f"  right  {_fmt(s['right'])}")
    print(f"  nearest anywhere in band: {_fmt(s['min'])}")
    print()
    print(ascii_map(w, h, a))
    return 0


if __name__ == "__main__":
    sys.exit(main())
