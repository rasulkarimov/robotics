#!/usr/bin/env python3
"""orbit.py -- move ONE object around the arm in N pick-and-place hops, verifying each
grasp honestly and adapting the grasp height from what actually happens.

WHY THIS EXISTS: the raw pieces (pick_eye vision servo, grab2 two-stage grasp) work, but
running a multi-hop "move it around" by hand meant re-deriving the same things every time:
- the gripper servo reading LIES for a light/compressible object (it read ~618 "empty"
  while the held-test proved a rock-solid hold), so success MUST be judged by the held-test
  (wiggle the base, a held object barely moves in the wrist camera), never by servo 1.
- with the arm on the FLOOR (2026-07-15 config) the true closing height is a NEGATIVE
  command z (~ -32..-40), nothing like the chassis-era rig.GRASP_Z. If a grab closes on
  air, the fix is almost always "go a few mm lower", so this tool does that automatically
  and REMEMBERS the height that worked for the rest of the run.

Reuses grab2.grab / grab2.put (they already handle wide-FOV aiming + timid bottom pass).
Run under system python3 (cv2/numpy); it shells out to ./arm (sudo+venv) for the arm.
"""
import sys, os, math, time, subprocess, argparse
sys.path.insert(0, "/home/astra/tools")
import numpy as np, cv2
import kin, rig, pick, pick_eye as pe, grab2

ARM = "/home/astra/robotics/arm"
CAR = "/home/astra/robotics/car.py"


def sh(args, ms=1000):
    subprocess.run([ARM, "step", args, "/tmp/x.jpg", str(ms)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)


def get_servo(j):
    out = subprocess.run(["sudo", "/home/astra/tools/venv/bin/python3",
                          "/home/astra/robotics/arm.py", "get", str(j)],
                         capture_output=True, text=True)
    import re
    m = re.search(r"(\d+)", out.stdout)
    return int(m.group(1)) if m else -1


def look(base, R, z, ms=1400):
    """Aim the wrist camera down at (base,R) from height z. base is pinned."""
    a = kin.s2a(base, 6); x, y = R * math.cos(a), R * math.sin(a)
    s = kin.ik_search(x, y, z, pitch_lo=155, pitch_hi=192, prefer=178)
    if not s:
        return False
    s[6] = base
    sh(",".join(f"{j}:{s[j]}" for j in (6, 5, 4, 3)), ms)
    return True


def snap(path):
    subprocess.run([CAR, "snapshot", path],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def held_shift():
    """Wiggle the base +-18 units and back; return object's px shift in the wrist view.
    A truly held object moves ~0 px (it is rigid with the camera); a floor object slides
    tens of px. Returns None if the object can't be seen (also a failure)."""
    base = get_servo(6)
    b = pe.see()
    if b is None:
        return None
    sh(f"6:{base-18}", 800); time.sleep(0.35)
    a = pe.see()
    sh(f"6:{base}", 800)
    if a is None:
        return None
    return math.dist(a, b)


def center_base(base, R, tol=45):
    """Rotate the (camera) base so the object sits near frame-centre, and return that base.
    This is the fix for the wide-angle alignment failures: grab2 aims from a FIXED base, so
    if the object has drifted sideways the servo starts too far off and hits the zone edge.
    Centre first, then grab2 only has to fix radius. servo6 UP => claw/camera swing LEFT, so
    an object on the LEFT (cx<240) needs base to INCREASE. ~0.35 units/px in this view.
    Sweeps to re-find the object if it is not in view (a failed grab may have nudged it)."""
    for _ in range(5):
        look(base, R + 10, 95, 1100)
        p = pe.see()
        if p is None:
            found = False
            for db in (45, -45, 90, -90, 135, -135):
                nb = max(320, min(660, base + db))
                look(nb, R + 10, 95, 1100)
                if pe.see() is not None:
                    base = nb; found = True; break
            if not found:
                return None
            continue
        cx = p[0]
        if abs(cx - 240) <= tol:
            return base
        base = int(max(320, min(660, base + round((240 - cx) * 0.35))))
    return base


def grab_verified(base, R, gz_state):
    """Centre-then-grab with an HONEST held-test, and the right fix per failure mode:
      - jaws still OPEN after grab2  => it never closed (alignment/reach) => RE-CENTRE, do
        NOT lower the height (lowering was the old bug that just nudged the object).
      - jaws CLOSED but not held      => closed on air => lower the closing height 6 mm.
    gz_state is a 1-element list (persists the learned height across hops).
    Returns (ok, base_used, shift)."""
    for attempt in range(6):
        cb = center_base(base, R)
        if cb is None:
            print("    [grab] object not found in sweep")
            return False, base, None
        base = cb
        rig.GRASP_Z = gz_state[0]
        ok = grab2.grab(base, R)
        g = get_servo(1)
        if g < 400:                      # jaws open ~157 => never closed
            print(f"    [grab] did not close (gripper={g}, jaws open) "
                  f"=> re-centre & retry, keep z={gz_state[0]:.0f}")
            continue
        shift = held_shift()
        if shift is not None and shift < 18.0:
            print(f"    [grab] HELD z={gz_state[0]:.0f} shift={shift:.1f}px gripper={g}")
            return True, base, shift
        print(f"    [grab] closed on air (shift={shift} gripper={g}) at "
              f"z={gz_state[0]:.0f} -> lower 6mm")
        sh(f"1:{pe.OPEN}", 500)
        if gz_state[0] - 6.0 >= rig.GRASP_Z_floor_min:
            gz_state[0] -= 6.0
        else:
            print("    [grab] at floor-height limit; stop lowering")
            return False, base, shift
    return False, base, None


def locate(base_lo=380, base_hi=880, step=25, R=150.0, z=None, target_cx=240):
    """Find the base angle that points the camera at the blue object, by sweeping the front
    arc and picking the base where the blob sits nearest frame-centre. Returns (base, R) or
    None. This replaces the hand-rolled pre-sweep I used to do by hand (which looked like a
    freeze to the user): give orbit `--start auto` and it finds the object itself. R is a
    default; grab2's visual servo refines radius from there, and center_base refines base.
    LESSON 2026-07-15: don't pre-abort on apparent orientation. A 3:1 bar that looked
    'elongated across the jaws' in the oblique look-view still grabbed and HELD (0.1px) —
    the shallow look-view foreshortens radially and exaggerates tangential length. The
    held-test is the only arbiter; let the grab run and judge by that.

    Range/height widened and re-derived 2026-07-19 after the arm was remounted on the car
    chassis: the old base_lo/hi=400-580 assumed a narrow forward arc and silently missed a
    real object sitting at base~700-800 (to the side); z=95 was a fixed height from before
    the remount and didn't match this session's re-measured floor (rig.GRASP_Z=-65) - z now
    defaults to rig.GRASP_Z+60 (the same HIGH convention grab2/tanggrab use) so it stays
    correct if GRASP_Z is re-measured again. A single coarse pass at one R is enough (the
    blob's pixel position moves smoothly and monotonically with base) - don't grid-search
    R too, that's needlessly slow for what a smooth 1D sweep already finds in one pass."""
    if z is None:
        z = rig.GRASP_Z + 60.0
    best = None  # (abs_err, base, cx)
    for base in range(base_lo, base_hi + 1, step):
        if not look(base, R, z, 1000):
            continue
        time.sleep(0.2)
        p = pe.see()
        if p is None:
            continue
        err = abs(p[0] - target_cx)
        if best is None or err < best[0]:
            best = (err, base, p[0])
    if best is None:
        return None
    print(f"    [locate] object at base~{best[1]} (cx={best[2]:.0f}); using R={R:.0f}")
    return (best[1], R)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True,
                    help="base:R where the object is now, or 'auto' to locate it, or "
                         "'locate' to only print the found position and exit (no motion)")
    ap.add_argument("--waypoints", default=None,
                    help="comma list base:R of placement targets, one per hop "
                         "(not needed for --start locate)")
    ap.add_argument("--gz", type=float, default=-35.0, help="initial closing height")
    ap.add_argument("--gzmin", type=float, default=-50.0, help="lowest allowed closing z")
    ap.add_argument("--outdir", default="/home/astra/robotics/orbit_out")
    args = ap.parse_args()

    rig.GRASP_Z_floor_min = args.gzmin
    os.makedirs(args.outdir, exist_ok=True)
    parse = lambda s: (int(s.split(":")[0]), float(s.split(":")[1]))
    if args.start in ("auto", "locate"):
        cur = locate()
        if cur is None:
            print("    [locate] blue object not found in front arc — is it in reach/lit?")
            return
        if args.start == "locate":
            print(f"located: base{cur[0]} R{cur[1]:.0f}")
            return
    else:
        cur = parse(args.start)
    if not args.waypoints:
        ap.error("--waypoints is required unless --start locate")
    wps = [parse(w) for w in args.waypoints.split(",")]
    gz = [args.gz]

    log = []
    for i, tgt in enumerate(wps, 1):
        print(f"\n=== HOP {i}/{len(wps)}: grab @ ~base{cur[0]} R{cur[1]:.0f} "
              f"-> place @ base{tgt[0]} R{tgt[1]:.0f} ===")
        ok, base_used, shift = grab_verified(cur[0], cur[1], gz)
        if not ok:
            print(f"    HOP {i} ABORTED (could not verify grasp)")
            log.append((i, cur, tgt, "GRAB_FAIL", gz[0], shift, None))
            look(base_used, cur[1] + 10, 95)
            continue
        grab2.put(tgt[0], tgt[1])
        # confirm placement + photo
        look(tgt[0], tgt[1] + 10, 95, 1500); time.sleep(0.4)
        photo = f"{args.outdir}/hop{i}_base{tgt[0]}.jpg"
        snap(photo)
        p = pe.see()
        print(f"    placed; object detected at {('%.0f,%.0f'%p) if p else 'NONE'} ; photo {photo}")
        log.append((i, cur, tgt, "OK", gz[0], shift, p))
        cur = tgt

    print("\n===== SUMMARY =====")
    print(f"final working GRASP_Z = {gz[0]:.0f}")
    for row in log:
        i, c, t, st, z, s, p = row
        print(f"hop{i}: {st:9s} grab(base{c[0]},R{c[1]:.0f}) place(base{t[0]},R{t[1]:.0f}) "
              f"z={z:.0f} held_shift={s} placed_px={p}")
    okc = sum(1 for r in log if r[3] == "OK")
    print(f"{okc}/{len(wps)} hops succeeded")


if __name__ == "__main__":
    main()
