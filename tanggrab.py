#!/usr/bin/env python3
"""tanggrab.py -- grab an elongated object no matter which way it lies, by rotating the
wrist (servo2) so the jaws close across the object's SHORT axis.

WHY THIS EXISTS: grab2/orbit close the jaws in a FIXED (tangential) direction. That grabs a
bar only if its long axis happens to point radially. A bar lying TANGENTIALLY (long axis
across the jaw-close line) used to be ungrabbable -- the jaws closed along its length. The
fix the user taught 2026-07-15: rotate the wrist 90deg so the jaws close across the short
axis. Verified (held 3.8px). See memory [[pick-place-orbit-lessons]].

ORDER MATTERS (each mistake below cost a failed attempt):
  1. AIM with NEUTRAL jaws (servo2=499). Rotated OPEN jaws swing into the top of the frame
     and OCCLUDE the object, so grab2's see() returns None before it descends. Aim clean.
  2. Get a good starting R FIRST (R-scan). A bad R puts the bar at the frame's top edge; the
     top pass then servos a CLIPPED blob whose centroid jumps and loses the object. Scan R
     until the blob's cy ~ GRASP_PIXEL.y, then servo from that clean mid-frame blob.
  3. ROTATE the wrist IN THE AIR (at HIGH, above the object), never at floor height --
     rotating open jaws at the floor sweeps and knocks the bar away.
  4. Descend, CLAMP, lift, held-test (the only honest success check).

Run under system python3 (cv2/numpy); shells out to ./arm for the arm, like orbit.py.
"""
import sys, os, math, time, argparse
sys.path.insert(0, "/home/astra/tools")
import numpy as np, cv2
import kin, rig, pick, pick_eye as pe, grab2, orbit

NEUTRAL = 499                 # jaws close tangentially (horizontal in image)
ROT_90 = NEUTRAL + 360        # +90deg -> jaws close radially (360 units = 90deg)
DEG2UNIT = 4.0                # 1000 servo units / 250 deg (arm.py: 0-1000 = -125..+125deg)

# The camera sits OFF TO THE SIDE of the jaws, so rotating the wrist SHIFTS the point where
# the jaws actually close, in the image, relative to the neutral GRASP_PIXEL. Aiming the
# bar's centre at GRASP_PIXEL therefore grabbed it RIGHT-of-centre (user caught this).
# Calibrated live 2026-07-15 (grab at several aim offsets, measured bar overhang): at a
# +90deg wrist rotation the rotated grasp centre is ~+60 px in x from GRASP_PIXEL. The shift
# scales ~linearly with the rotation angle (offset traces an arc; the x-component dominates
# and the y-component is within the ~+-10px calibration noise, so we model it as x-only and
# verify by overhang). So aim +ROT_DX_90*(deg/90) px in x. Only applied when rotating.
ROT_DX_90, ROT_DY_90 = 60.0, 0.0
ROT_AIM_DX, ROT_AIM_DY = ROT_DX_90, ROT_DY_90   # back-compat alias (the +90deg values)

# REACH pull-in for a rotated grasp. Turning the wrist swings the jaw-close point RADIALLY
# OUTWARD, so a rotated grab closes ~cm PAST the object (user saw ~5cm at gz=-30, "уходишь за
# брус, он остаётся под тобой"). The pixel aim offset above only fixes the TANGENTIAL part; the
# radial part is uncompensated. Fix: after aiming+rotating, pull the arm IN by
# PULLIN_PER_DEG*|rotation_deg| mm before descending. Calibrated 2026-07-16 at gz=-30: a +47deg
# grab needed 40mm to centre (held 0.3px) -> ~0.85 mm/deg. Re-check if gz or the rig changes.
PULLIN_PER_DEG = 40.0 / 47.0


def wrist_for_bar(long_ang, aspect):
    """Pick the wrist servo2 (and aim offset) that closes the jaws ACROSS the bar's SHORT
    axis, for ANY bar orientation -- not just the old binary neutral/90deg. long_ang is the
    measure() convention: 0/180 = tangential (horizontal in image), 90 = radial (vertical).
    Neutral jaws close horizontally, so the jaw line must sit at long_ang+90 (perpendicular);
    the wrist rotation from neutral is that angle wrapped into (-90, 90]. Returns
    (servo2, aim_dx, aim_dy). Anchors preserved: long_ang~0 -> +90deg (servo 859, the old
    tangential case); long_ang~90 -> neutral (servo 499, the radial case)."""
    if aspect < 1.8:                 # too square to trust a long axis -> don't rotate
        return NEUTRAL, 0.0, 0.0
    a = long_ang + 90.0              # perpendicular jaw line, in degrees
    while a > 90.0:  a -= 180.0      # minimal-rotation representative, half-open at +90
    while a <= -90.0: a += 180.0
    servo2 = int(round(NEUTRAL + a * DEG2UNIT))
    return servo2, ROT_DX_90 * (a / 90.0), ROT_DY_90 * (a / 90.0)


def measure(img):
    """Return (cx, cy, aspect, long_axis_deg) of the blue blob, or None.
    long_axis_deg: 0/180 = horizontal (tangential), 90 = vertical (radial)."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, pe.OBJ_LO, pe.OBJ_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    (cx, cy), (w, h), ang = cv2.minAreaRect(c)
    aspect = max(w, h) / max(1.0, min(w, h))
    long_ang = (ang if w >= h else ang + 90) % 180
    return cx, cy, aspect, long_ang


R_MAX = 200.0   # raised 178->200 on 2026-07-16 with the wider pitch band (pick_eye.PITCH_BAND
                # 22): the HIGH pose now reaches ~R200 by leaning the wrist, so a far object can
                # be centred+grabbed instead of sitting clipped at the frame top. Floor descent
                # is reachable to ~R200 too (max_reach at z=-30 is ~254).

# grab-frame gains, measured live 2026-07-15 (differ from the LOOK frame, and are the whole
# reason locate's base is wrong for grabbing -- FRAME MISMATCH). Higher base -> object moves
# RIGHT (~2.5 px/base-unit); higher R -> object moves DOWN (~4.5 px/R-unit).
#
# Observed 2026-07-19 (post chassis-remount): center_grabframe() sometimes doesn't converge
# within maxit and settles off-target (once landed with the blob clipped at the frame edge,
# aspect/angle unreadable). Didn't re-derive these gains this session - if this keeps
# happening, re-measure them fresh rather than trusting they're still right after the
# remount, the same way rig.GRASP_Z needed re-measuring (see rig.py).
BASE_PER_PX = 1 / 2.5
R_PER_PX = 1 / 4.5


def find_R(base, HIGH, lo=118, hi=200, step=8):
    """Sweep R at a fixed base to FIND the object in frame and vertically centre it (blob cy
    nearest GRASP_PIXEL.y). Needed because a far-drifted object sits ABOVE the frame at small
    R -- you must INCREASE R to bring it into view, so a plain nudge-loop that shrinks R on a
    miss walks the wrong way and loses it. Returns (R, p) or None."""
    best = None
    for RR in range(lo, hi + 1, step):
        if grab2.pose(base, float(RR), HIGH, 850) is None:
            continue
        pe.arm_step(f"1:{pe.OPEN}", 420); time.sleep(0.28)
        p = pe.see()
        if p is None:
            continue
        err = abs(p[1] - rig.GRASP_PIXEL[1])
        if best is None or err < best[0]:
            best = (err, float(RR), p)
    return None if best is None else (best[1], best[2])


def center_grabframe(base, R, HIGH, tol=42, maxit=6):
    """Centre the object on GRASP_PIXEL in the GRAB frame: (1) R-sweep to FIND it and centre
    vertically at this base, then (2) nudge base (from cx error) to centre horizontally,
    holding vertical with small R corrections. Fixes the LOOK-vs-GRAB frame mismatch that
    left the object far-right (broke a practice rep) AND the far-drift case (object above
    the frame). Returns (base, R, p) or None."""
    GX, GY = rig.GRASP_PIXEL
    hit = find_R(base, HIGH)
    if hit is None:                                    # not at this base -> sweep base wide
        for b in range(380, 601, 40):
            hit = find_R(b, HIGH)
            if hit:
                base = b; break
        if hit is None:
            return None
    R, p = hit
    for _ in range(maxit):
        ex, ey = p[0] - GX, p[1] - GY
        if abs(ex) <= tol and abs(ey) <= tol:
            return base, R, p
        # clamp widened 2026-07-19 (was 360-620, assumed the object stays in a narrow
        # forward arc; a real object at base~780 got clamped down to 620 every
        # correction step and was lost) - now just the arm's real servo travel limits.
        base = int(max(150, min(850, base + max(-45, min(45, (GX - p[0]) * BASE_PER_PX)))))
        R = max(120.0, min(R_MAX, R + max(-14, min(14, (p[1] - GY) * R_PER_PX))))
        if grab2.pose(base, R, HIGH, 1000) is None:
            R = max(120.0, R - 8); continue
        pe.arm_step(f"1:{pe.OPEN}", 420); time.sleep(0.28)
        p = pe.see()
        if p is None:                                  # lost it -> re-find via R-sweep
            hit = find_R(base, HIGH)
            if hit is None:
                return None
            R, p = hit
    return base, R, p


def tanggrab(base, R, gz, outdir):
    rig.GRASP_Z = gz
    HIGH = rig.GRASP_Z + 60.0
    os.makedirs(outdir, exist_ok=True)
    snap = lambda name: cv2.imwrite(f"{outdir}/{name}.jpg", pick.frame())

    pe.arm_step(f"2:{NEUTRAL}", 800); time.sleep(0.3)   # neutral jaws for clean aiming

    # 1) coarse-centre in the GRAB frame (fixes the LOOK-vs-GRAB frame mismatch: both base
    #    and R), so the fine servo starts from a clean, unclipped, roughly-centred blob
    hit = center_grabframe(base, R, HIGH)
    if hit is None:
        print("    [tang] object not found while centring"); return False
    base, R, p0 = hit
    print(f"    [tang] centred -> base={base} R={R:.0f} (blob {p0[0]:.0f},{p0[1]:.0f})")

    # measure orientation and pick the wrist angle that puts the jaws ACROSS the short axis
    # (works for any diagonal, not just the binary neutral/90deg case).
    _, _, aspect, long_ang = measure(pick.frame())
    rot, aim_dx, aim_dy = wrist_for_bar(long_ang, aspect)
    print(f"    [tang] aspect={aspect:.2f} long_axis={long_ang:.0f}deg -> wrist servo2={rot} "
          f"({(rot-NEUTRAL)/DEG2UNIT:+.0f}deg), aim +{aim_dx:.0f},{aim_dy:.0f}px "
          f"{'(NEUTRAL)' if rot==NEUTRAL else ''}")

    # 2) fine-aim with NEUTRAL jaws. When we'll rotate, aim the bar's CENTRE to the ROTATED
    #    grasp centre (GRASP_PIXEL + aim offset) so the rotated jaws land on the bar's centre,
    #    not off to one side. Done by temporarily overriding the servo's target pixel.
    a = kin.s2a(base, 6); x, y = R * math.cos(a), R * math.sin(a)
    base_gp = rig.GRASP_PIXEL
    if rot != NEUTRAL:
        rig.GRASP_PIXEL = (base_gp[0] + aim_dx, base_gp[1] + aim_dy)
    r = pe.servo(x, y, HIGH, iters=8, tol_px=18.0, label="прицел")
    aim_err = grab2.err_now()
    rig.GRASP_PIXEL = base_gp
    if r is None:
        print("    [tang] fine-aim lost object; using coarse-centred pose")
        x, y = R * math.cos(a), R * math.sin(a)
        if not pe.goto(x, y, HIGH, 900) or grab2.err_now() is None:
            print("    [tang] object gone"); return False
    else:
        x, y = r
    snap("tang_aimed")
    print(f"    [tang] aimed (rot-offset {'on' if rot!=NEUTRAL else 'off'}) "
          f"err={aim_err:.0f}px")

    # 3) rotate wrist IN THE AIR, pull the reach IN to cancel the radial jaw swing, then descend
    if rot != NEUTRAL:
        pe.arm_step(f"2:{rot}", 900); time.sleep(0.4); snap("tang_rotated_high")
        pull = PULLIN_PER_DEG * abs((rot - NEUTRAL) / DEG2UNIT)
        rc = math.hypot(x, y); s = max(0.1, (rc - pull) / rc)
        x, y = x * s, y * s
        print(f"    [tang] reach pull-in {pull:.0f}mm -> R {rc - pull:.0f}")
    if not pe.goto(x, y, rig.GRASP_Z, 1400):
        print("    [tang] descend unreachable"); return False

    # 4) close, lift, held-test
    pe.arm_step(f"1:{pe.CLAMP}", 900); time.sleep(0.3)
    g = orbit.get_servo(1)
    pe.goto(x, y, rig.GRASP_Z + 90, 1500); snap("tang_lifted")
    shift = orbit.held_shift(); snap("tang_after")
    held = shift is not None and shift < 18.0
    print(f"    [tang] closed gripper={g}, held_shift={shift} -> "
          f"{'HELD ✓' if held else 'NOT held ✗'}")
    return {"held": held, "shift": shift, "base": base, "R": R, "x": x, "y": y, "rot": rot}


def place(x, y, gz):
    """Set the object down at (x,y), then reset the wrist to neutral. Pass a FIXED inner spot
    (not the grabbed pose) to stop the object walking outward over repeated grabs. KEY: the
    jaws OPEN (release) BEFORE the wrist resets to neutral, so the object lands at its held
    (rotated) orientation -- this is what keeps a diagonal bar diagonal across a reps drill."""
    rig.GRASP_Z = gz
    pe.goto(x, y, rig.GRASP_Z + 90, 1500)
    pe.goto(x, y, rig.GRASP_Z, 1400)
    pe.arm_step(f"1:{pe.OPEN}", 900); time.sleep(0.3)   # release at the rotated orientation
    pe.goto(x, y, rig.GRASP_Z + 90, 1400)
    pe.arm_step(f"2:{NEUTRAL}", 900)                     # only now reset wrist (bar already down)
    print(f"    [tang] placed at x={x:.0f} y={y:.0f}, wrist neutral")


def drill(base, drill_spot, gz0, reps, outdir):
    """Repeat grab+place `reps` times at a FIXED spot, re-measuring the bar's angle each rep
    (so the wrist turn self-corrects as the bar creeps). place() releases before resetting the
    wrist, so a diagonal bar stays diagonal for the next rep. Adaptive gz: if a grab isn't
    held, drop 6mm and retry once. Returns a list of per-rep result dicts. Validated 3/3 on a
    ~150deg bar 2026-07-16 (held-shifts 0.2/0.3/5.5px)."""
    db, dr = (int(drill_spot.split(":")[0]), float(drill_spot.split(":")[1]))
    a = kin.s2a(db, 6); dx, dy = dr * math.cos(a), dr * math.sin(a)
    out = []
    for rep in range(1, reps + 1):
        print(f"\n=== REP {rep}/{reps} ===")
        gz = gz0
        res = None
        for _ in range(2):
            res = tanggrab(base, 130.0, gz, outdir)
            if res and res.get("held"):
                break
            print(f"    [drill] not held at gz={gz}; lowering 6mm, retrying"); gz -= 6.0
        out.append(res)
        if not (res and res.get("held")):
            print(f"    [drill] rep {rep} FAILED - stopping"); break
        place(dx, dy, gz)
        base = db                                       # subsequent reps grab from the drill spot
    print("\n=== DRILL SUMMARY ===")
    for i, r in enumerate(out, 1):
        print(f"rep{i}: held={r and r.get('held')} shift={r and r.get('shift')} "
              f"rot={r and r.get('rot')}")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="auto", help="object base angle, or 'auto' to locate")
    ap.add_argument("--gz", type=float, default=-30.0,
                    help="grasp close height. -30 (was -35): user 2026-07-16 saw the jaws catch "
                         "the floor closing at -35, asked for +4-5mm. Adaptive drop still kicks "
                         "in 6mm at a time if a grab closes on air.")
    ap.add_argument("--place", action="store_true",
                    help="after the held-test, set the object down at --reset-to")
    ap.add_argument("--reset-to", default="445:128",
                    help="base:R to place the object at (a FIXED inner spot so it doesn't "
                         "drift outward over repeats). Default 445:128 (centred, in reach).")
    ap.add_argument("--reps", type=int, default=1,
                    help="run a grab+place DRILL this many times at --drill (re-measures the "
                         "angle each rep so the wrist self-corrects). >1 implies placing.")
    ap.add_argument("--drill", default="460:140",
                    help="base:R fixed spot the reps drill grabs from / places to.")
    ap.add_argument("--outdir", default="/home/astra/robotics/orbit_out")
    args = ap.parse_args()

    if args.base == "auto":
        loc = orbit.locate()
        if loc is None:
            print("    [tang] no blue object found"); return
        base, R = loc
    else:
        base, R = int(args.base), 130.0

    if args.reps > 1:
        drill(base, args.drill, args.gz, args.reps, args.outdir)
        return

    res = tanggrab(base, R, args.gz, args.outdir)
    if res and res.get("held") and args.place:
        rb, rr = args.reset_to.split(":")
        a = kin.s2a(int(rb), 6)
        place(float(rr) * math.cos(a), float(rr) * math.sin(a), args.gz)
    print("RESULT:", res)


if __name__ == "__main__":
    main()
