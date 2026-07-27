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

Run under the VENV python (/home/astra/tools/venv/bin/python3 - cv2/numpy live there,
NOT in system python); shells out to ./arm for the arm, like orbit.py.
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


# Measured 2026-07-27 on this workspace, and EXTENT is the discriminator that works:
#            area    aspect  extent  solidity
#   bar     21377     3.45    0.78     0.84
#   glow     9549     3.46    0.31     0.53     <- long and thin like the bar; extent kills it
#   sockets  1905     1.71    0.70     0.90     <- solidity as HIGH as the bar's
#   LED      1266     1.07    0.40     0.64
# Solidity does NOT separate them (impostors reach 0.88-0.90 while the clipped bar is only
# 0.84) - an earlier 0.90 threshold rejected the real bar and found nothing at all. Extent
# separates cleanly at 0.75: every impostor is <= 0.70, the bar is 0.78.
# Thresholds set from BOTH a bar view and a no-bar view, because a single frame is not
# enough: calibrated on one frame (extent 0.78) the cut sat at 0.75, and a second, blurrier
# view of the same bar measured 0.62 and was REJECTED - measure() then returned None while
# see() still saw the bar, so grasp_bar aborted every attempt without even trying.
#            area    aspect  extent  solidity
#   bar     21377     3.45    0.78     0.84
#   bar     27247     2.98    0.62     0.75   <- same bar, different view
#   glow     9549     3.46    0.31     0.53
#   sliver    797     2.94    0.43     0.58
BAR_MIN_EXTENT = 0.50      # contour area / minAreaRect area
BAR_MIN_SOLIDITY = 0.65    # contour area / convex-hull area


# Mean HSV saturation inside the contour, and it is the ONE feature that separates the bar
# from the power strip's blue glow cleanly. Shape does not: the glow is long, thin and can
# look as bar-like as the bar itself, and thresholds tight enough to reject it also rejected
# real bar views (measured extent 0.62-0.78 for the bar vs 0.31-0.43 for glow - overlapping
# once blur is involved). Measured mean saturation, though:
#     bar   112, 123      <- pencil-shaded blue, PALE (measured far/normal range, R>=140mm)
#     LEDs  155
#     glow  166, 227      <- an emitter, vivid
# So the test is an UPPER bound, which is the opposite of the intuition that "the real object
# is the more colourful one". Scene-specific: it holds because this bar is coloured in by
# hand. A glossy, vividly-blue object would need this raised or replaced.
#
# SATURATION IS ALSO DISTANCE-DEPENDENT, and the original 140 didn't account for it. Measured
# 2026-07-27: at the normal R>=140mm hover the bar reads 105-124 (matches calibration above),
# but after centring pulls R below ~140mm (which the corrected BASE_PER_PX/R_PER_PX gains and
# a live-measured GRASP_PIXEL now legitimately do - see rig.py) the SAME bar read 172-173,
# six calls in a row, and got rejected as "glow" every time - 0/4 held in a drill, with no
# grasp even attempted (measure() returning None aborts before aim/clamp). The rejected blob's
# extent (0.72-0.92) and solidity (0.87-0.92) matched the BAR signature exactly, nothing like
# glow's 0.31-0.53/0.53-0.75 - so raising the ceiling here does not reopen the glow false-
# positive, it only affects a case shape already disambiguates. Raised with margin above the
# observed close-range peak; re-verify if grasps start closing on the strip again.
BAR_MAX_SATURATION = 190


def _mean_saturation(c):
    import numpy as _np
    x, y, w, h = cv2.boundingRect(c)
    if w <= 0 or h <= 0:
        return 255.0
    img = _LAST_FRAME_HSV
    if img is None:
        return 0.0
    mask = _np.zeros(img.shape[:2], _np.uint8)
    cv2.drawContours(mask, [c], -1, 255, -1)
    vals = img[:, :, 1][mask > 0]
    return float(vals.mean()) if vals.size else 255.0


_LAST_FRAME_HSV = None


def _pick_bar_contour(cnts):
    """Largest contour that is actually BAR-SHAPED, or None.

    "Biggest blue blob" is not good enough here: the power strip's indicator LEDs and its
    bluish sockets pass the hue filter and sit permanently in the workspace. Measured live
    2026-07-27 with no bar in frame at all - blobs of 531-4444 px, aspect 1.03-1.66 - and
    they were being measured as "the bar", which fed a garbage long_ang/aspect into
    wrist_for_bar (whose aspect < 1.8 guard then quietly disabled the wrist rotation) and
    sent the arm off to grasp bare floor. The bar is long; the impostors are round."""
    best = None
    for c in cnts:
        a = cv2.contourArea(c)
        if a < pe.OBJ_SHAPE_MIN_AREA:      # same bulk requirement see() uses
            continue
        (_, _), (w, h), _ = cv2.minAreaRect(c)
        if max(w, h) / max(1.0, min(w, h)) < pe.OBJ_MIN_ASPECT:
            continue
        # SOLIDITY/EXTENT. Elongation alone is not enough: the blue GLOW that spills along
        # the power cable is long and thin too (measured aspect 3.5-7.0 with no bar in
        # frame). A real bar fills its bounding rectangle and is convex; a glow is diffuse
        # and ragged. Measured on the impostors: extent 0.31-0.70, solidity 0.53-0.90.
        if a / max(1.0, w * h) < BAR_MIN_EXTENT:
            continue
        hull = cv2.convexHull(c)
        if a / max(1.0, cv2.contourArea(hull)) < BAR_MIN_SOLIDITY:
            continue
        if _mean_saturation(c) > BAR_MAX_SATURATION:
            continue
        if best is None or a > cv2.contourArea(best):
            best = c
    return best


def measure(img):
    """Return (cx, cy, aspect, long_axis_deg) of the blue blob, or None.
    long_axis_deg: 0/180 = horizontal (tangential), 90 = vertical (radial)."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    globals()["_LAST_FRAME_HSV"] = hsv
    m = cv2.inRange(hsv, pe.OBJ_LO, pe.OBJ_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = _pick_bar_contour(cnts)
    if c is None:
        return None
    (cx, cy), (w, h), ang = cv2.minAreaRect(c)
    aspect = max(w, h) / max(1.0, min(w, h))
    long_ang = (ang if w >= h else ang + 90) % 180
    return cx, cy, aspect, long_ang


def measure_full(img, margin=3):
    """measure(), plus the bar's GEOMETRIC centre and whether the blob is CLIPPED by the frame.

    Returns (cx, cy, aspect, long_ang, clipped) or None. cx,cy is the minAreaRect centre -
    for a uniform bar that is its centre of mass, and it is what you want the jaws to close
    on. The connected-component CENTROID that pick_eye.see() returns is NOT the same thing
    once part of the bar leaves the frame: it slides toward whichever end is still visible,
    so the jaws land near an end instead of the middle (user, watching: "иногда ты хватаешь
    за край бруса"). Gripping an end is bad for a bar and fatal for a small USB plug, which
    is the whole reason this matters.

    `clipped` is true when the contour touches the frame border, i.e. part of the object is
    out of view and BOTH centre estimates are untrustworthy - back off / re-centre first
    rather than aiming at a number that cannot be right. This is also the condition behind
    the garbage aspect readings (measured 1.32 on a visibly 4:1 bar) that silently disable
    the wrist rotation via wrist_for_bar's aspect<1.8 guard."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    globals()["_LAST_FRAME_HSV"] = hsv
    m = cv2.inRange(hsv, pe.OBJ_LO, pe.OBJ_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = _pick_bar_contour(cnts)
    if c is None:
        return None
    (cx, cy), (w, h), ang = cv2.minAreaRect(c)
    aspect = max(w, h) / max(1.0, min(w, h))
    long_ang = (ang if w >= h else ang + 90) % 180
    x, y, bw, bh = cv2.boundingRect(c)
    H, W = m.shape[:2]
    clipped = (x <= margin or y <= margin or x + bw >= W - margin or y + bh >= H - margin)
    return cx, cy, aspect, long_ang, clipped


def see_centre():
    """pick_eye.see()-compatible probe that returns the bar's MIDDLE, not its centroid.

    Returns the minAreaRect centre EVEN WHEN CLIPPED. An earlier version returned None on
    any clipping so the loop wouldn't chase a wrong point - that was a bad trade and the
    log says so: the bar touches the frame edge in most HIGH-pose frames, so the aim loop
    kept treating a perfectly visible bar as lost. Success fell 4/4 -> 1/4 and reps went
    from ~50 s to ~180 s. A clipped rect centre is still a far better target than the
    centroid (which slides toward the visible end); `clipped` stays available from
    measure_full() as a diagnostic and for the aspect guard."""
    import pick
    m = measure_full(pick.frame())
    if m is None:
        return None
    return float(m[0]), float(m[1])


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
# RE-MEASURED LIVE 2026-07-27 (nudge the arm, watch the blob move), and both were wrong:
#   base: +25 units moved the blob +122 px  ->  4.9 px/unit, NOT the assumed 2.5
#   R:    -15 mm   moved the blob -34.4 px  ->  2.29 px/mm,  NOT the assumed 4.5
# The base gain being 2x too big made every horizontal correction OVERSHOOT, so centring
# oscillated and walked away instead of converging (user: "почему твой поиск всегда уходит
# слишком далеко?").
BASE_PER_PX = 1 / 4.9
R_PER_PX = 1 / 2.29


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
        # SIGN: increasing R moves the blob DOWN the frame (measured above), so to pull a blob
        # that sits ABOVE the target (p[1] < GY) down onto it, R must GROW - the error term is
        # (GY - p[1]), not (p[1] - GY). The old sign drove R the wrong way on every vertical
        # correction, walking the object further off-frame each iteration (user: "в
        # противоположную сторону уходишь, когда известно в какой стороне брус").
        R = max(120.0, min(R_MAX, R + max(-14, min(14, (GY - p[1]) * R_PER_PX))))
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

    # 3) rotate wrist IN THE AIR, then descend
    # Pull-in DISABLED 2026-07-19: PULLIN_PER_DEG was calibrated 2026-07-16 at gz=-30, a
    # different grasp height than this session's re-measured rig.GRASP_Z (see rig.py) - with
    # the current calibration it consistently dragged an already-good aim (6-13px error)
    # into a bad one (100-200+px, failed grasp), confirmed live by the user watching. Descend
    # straight down at the aimed (x,y) instead - verified 4/4 held across a supervised run
    # plus 3 unsupervised reps. If PULLIN_PER_DEG is ever re-measured for a fresh GRASP_Z,
    # this can be re-enabled.
    if rot != NEUTRAL:
        pe.arm_step(f"2:{rot}", 900); time.sleep(0.4); snap("tang_rotated_high")
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


def wiggle_held_test(log=print):
    """The honest grasp check: swing the base a real amount and see if the object's pixel
    moves. A held object is rigid to the wrist camera (a few px); one still on the floor
    slides 50-300px from parallax. Returns (held, shift1, shift2). Trust THIS, never the
    gripper servo reading (proven to overlap between hit and miss)."""
    b0 = orbit.get_servo(6)
    p1 = pe.see()
    pe.arm_step(f"6:{b0 + 40}", 700); time.sleep(0.3)
    p2 = pe.see()
    pe.arm_step(f"6:{b0 - 20}", 700); time.sleep(0.3)
    p3 = pe.see()
    s1 = math.dist(p1, p2) if (p1 and p2) else None
    s2 = math.dist(p2, p3) if (p2 and p3) else None
    held = s1 is not None and s1 < 15 and s2 is not None and s2 < 15
    log(f"    [wiggle] shifts=({s1}, {s2}) -> {'HELD' if held else 'NOT held'}")
    return held, s1, s2


def open_jaws_gently(log=print):
    """Open the jaws only AFTER lowering to floor level, so anything still held is SET DOWN
    instead of dropped from height.

    Opening at whatever height the arm happens to be at is how a drill turns into a game of
    fetch: the bar falls ~9 cm, bounces, and lands somewhere new, which invalidates the hint
    and sends the next attempt hunting. The user had to say it twice - the second time as
    "Хватит бросать брус!!!" - because the first fix only covered the failed-wiggle path and
    missed this one, at the top of every attempt.

    Works from the CURRENT pose (read back from the servos), because this runs before the
    object has been located and there is no target x,y yet."""
    try:
        s3, s4, s5, s6 = (orbit.get_servo(j) for j in (3, 4, 5, 6))
        x, y, _ = kin.fk(s5, s4, s3, s6)
        pe.goto(x, y, rig.GRASP_Z, 1200)
        time.sleep(0.2)
    except Exception as e:                     # never let a tidy-up step abort the grasp
        log(f"    [grasp_bar] could not lower before opening ({e}); opening in place")
    pe.arm_step(f"1:{pe.OPEN}", 700)


def grasp_bar(hint_base, hint_R, gz=None, retries=1, log=print):
    """The canonical end-to-end bar grasp, as proven in the 2026-07-19 drills (19/20 held
    across angles 57-158deg and reaches 130-190mm when counting the auto-retry):

    locate_near(hint) -> center_grabframe -> measure orientation -> rotated fine-aim ->
    rotate wrist in the air -> descend STRAIGHT DOWN (no pull-in, see note in tanggrab())
    -> clamp -> lift -> wiggle_held_test -> on failure, retry once from the same hint
    (the observed failure mode is a garbage one-off orientation measurement, not anything
    persistent - a plain retry fixed it in the drill).

    Returns the result dict (held/base/R/x/y/rot) or None if not found/unreachable.
    Leaves the bar HELD and lifted on success; escalate to a human after this returns
    a not-held result (i.e. after the built-in retry already failed)."""
    if gz is not None:
        rig.GRASP_Z = gz
    HIGH = rig.GRASP_Z + 60.0
    # Last wiggle-test result seen this call, kept across attempts so a final failure can
    # still report HOW CLOSE it was (train_grasp.py logged nothing at all for failed reps
    # before this - every miss looked identical whether it slid 20px or 200px, which made
    # "is the closing moment borderline or just wrong" unanswerable from the log).
    last_shifts = (None, None)
    last_aim_err = None
    # Where the jaws really close, measured now - the stored constant goes stale across
    # mountings and steers every grasp off to one side while the aim loop still reports
    # clean convergence. One move, and it removes the whole failure mode. See rig.py.
    for attempt in range(1 + retries):
        if attempt:
            log(f"    [grasp_bar] retry {attempt}/{retries} from the same hint")
        pe.arm_step(f"2:{NEUTRAL}", 500)
        open_jaws_gently(log=log)
        loc = orbit.locate_near(hint_base, hint_R, log=log)
        if loc is None:
            continue
        # Measure the closing point BEFORE centring, at the located pose (already HIGH over
        # the object). Two constraints pin it here:
        #  - it must NOT be measured at whatever pose the arm was in on entry: that made it
        #    swing ~100 px between reps - (252,265) (224,229) (217,314) (202,320) - and every
        #    one of those attempts failed. The claw is only "fixed in the image" for a GIVEN
        #    pose, so the measurement has to share the working pose.
        #  - it must come BEFORE center_grabframe, because centring STEERS TOWARDS
        #    rig.GRASP_PIXEL. Measuring afterwards left centring chasing the stale (170,146)
        #    while the truth was (184,353); it drove R down to its 120 mm clamp - past the
        #    ~140 mm floor where the jaws start catching the ultrasonic bracket - and still
        #    ended 73 px off.
        gp = pe.measure_grasp_pixel()
        log(f"    [grasp_bar] closing point {'measured ' + str(tuple(round(v) for v in gp)) if gp else 'MEASURE FAILED, using stored ' + str(rig.GRASP_PIXEL)}")
        hit = center_grabframe(loc[0], loc[1], HIGH)
        if hit is None:
            continue
        base, R, _ = hit
        m = None
        for _ in range(6):
            m = measure(pick.frame())
            if m:
                break
            time.sleep(0.2)
        if m is None:
            continue
        _, _, aspect, long_ang = m
        rot, aim_dx, aim_dy = wrist_for_bar(long_ang, aspect)
        a = kin.s2a(base, 6); x, y = R * math.cos(a), R * math.sin(a)
        base_gp = rig.GRASP_PIXEL
        if rot != NEUTRAL:
            rig.GRASP_PIXEL = (base_gp[0] + aim_dx, base_gp[1] + aim_dy)
        # Aim with the blob CENTROID (pe.see, the default). Aiming at the minAreaRect centre
        # via see_centre was tried 2026-07-26 to stop the jaws landing near a bar's end, and
        # it REGRESSED hard: 4/4 held -> 1/4, and reps went ~50 s -> ~200 s. Re-verified with
        # a correct hint and a clean, fully-visible bar (aspect 6.5), so it was the aim point
        # itself, not a stale hint or a clipped blob. The two points normally sit only ~4 px
        # apart, so the centroid is NOT what makes a grip land off-centre - look elsewhere
        # before retrying this. see_centre/measure_full are kept: they are the right tools for
        # MEASURING where the grip landed (off_centre_px in train_grasp.py) and for `clipped`.
        r = pe.servo(x, y, HIGH, iters=8, tol_px=18.0, label="aim")
        rig.GRASP_PIXEL = base_gp
        if r:
            x, y = r
        # How far off the aim actually landed, independent of whether the grip later held -
        # separates "aim converged but the jaws still slipped" (mechanical) from "aim never
        # got close" (still an aiming problem), which a bare held=False can't distinguish.
        p_final = pe.see()
        target = (base_gp[0] + aim_dx, base_gp[1] + aim_dy) if rot != NEUTRAL else base_gp
        last_aim_err = round(math.dist(p_final, target), 1) if p_final else None
        if rot != NEUTRAL:
            pe.arm_step(f"2:{rot}", 900); time.sleep(0.4)
        if not pe.goto(x, y, rig.GRASP_Z, 1400):
            log("    [grasp_bar] descend unreachable")
            continue
        pe.arm_step(f"1:{pe.CLAMP}", 900); time.sleep(0.3)
        pe.goto(x, y, rig.GRASP_Z + 90, 1500)
        held, s1, s2 = wiggle_held_test(log=log)
        last_shifts = (s1, s2)
        if held:
            return {"held": True, "shifts": (s1, s2), "long_ang": long_ang,
                    "base": base, "R": R, "x": x, "y": y, "rot": rot,
                    "attempts": attempt + 1, "aim_err": last_aim_err}
        # Put it DOWN before letting go. After a failed wiggle the arm is parked ~90 mm up,
        # and opening the jaws there drops the bar - it bounces and skitters to a new spot,
        # which invalidates the hint and makes the next attempt hunt for it (user, watching
        # a drill do this repeatedly: "Хватит бросать брус. Клади аккуратно, а то отскочет").
        # A partial grip is the common case here, so this runs on EVERY failed attempt.
        pe.goto(x, y, rig.GRASP_Z, 1200); time.sleep(0.2)
        pe.arm_step(f"1:{pe.OPEN}", 700); time.sleep(0.2)
        pe.goto(x, y, rig.GRASP_Z + 60, 1200)
        # ...and only then reset the wrist, so a caller that reads {"held": False} back
        # doesn't inherit a rotated claw it never asked for (bit a live session 2026-07-26:
        # the rotated jaws produced misleading frames for everything after).
        pe.arm_step(f"2:{NEUTRAL}", 500)
    return {"held": False, "attempts": 1 + retries, "shifts": last_shifts, "aim_err": last_aim_err}


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
    ap.add_argument("--gz", type=float, default=None,
                    help="grasp close height; default = rig.GRASP_Z (the calibrated floor). "
                         "Earlier hardcoded defaults (-35, then -30) went stale every time "
                         "the floor was re-measured - deriving from rig keeps one source of "
                         "truth. Adaptive drop still kicks in 6mm at a time on an air-close.")
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

    if args.gz is None:
        args.gz = rig.GRASP_Z

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
