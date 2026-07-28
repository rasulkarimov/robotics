#!/usr/bin/env python3
"""Measured facts about the CURRENT physical rig. Import this instead of guessing.

Every number here was measured on the real robot, with the user watching, because every
single one of them differs from what the model predicts. Re-measure them (and only them)
if the arm is remounted or the camera is moved.

    THE ARM NOW SITS ON THE CAR CHASSIS, 110 mm ABOVE THE FLOOR,
    AND THE CAMERA IS ON THE WRIST, LOOKING OUT BETWEEN THE JAWS.

Both facts invalidate everything calibrated before them.
"""

# --- geometry, in kin.py's COMMAND coordinates (not true millimetres - see kin.py) ---

ARM_ABOVE_FLOOR = 110.0     # arm base height above the floor, now that it's on the chassis

# THE NUMBER THAT MATTERS: descend to this and close, and the jaws meet exactly at floor
# level. Anything lying on the floor gets picked up; the claw does not ram the ground.
#
# IT IS NOT A CONSTANT. Measured -90 on one occasion and -75 on the next, with the same
# arm on the same flat floor - a 15 mm drift, which is more than the height of a thin
# object. The car apparently doesn't sit identically each time (tyres, a slight lean), and
# the arm's sag varies with pose. So: RE-MEASURE IT before a session that needs precision
# (close the jaws, step down 5 mm at a time until they touch), and don't trust a stale
# value. There is no automatic contact detector - the arm is strong enough to LIFT THE CAR
# rather than stall, so servo feedback stays clean while the wheels leave the ground.
GRASP_Z = -75.0        # re-measured 2026-07-27, AFTER the robot fell over and was stood back
                       # up. Was -65.0. Found from the user watching a drill ("ты примерно на
                       # 1 см выше делал хват") and confirmed by a clean sweep at one spot:
                       #   -55 -> wiggle 294/417 px (way off)
                       #   -65 -> wiggle 130-180 px (close, never held)
                       #   -75 -> wiggle 4.4/6.4 px, HELD first attempt
                       # Note the failure signature of grasping TOO HIGH looked exactly like
                       # bad aim: the aim loop converged to a few px and the object still
                       # slid away in the wiggle test. Height and aim are NOT distinguishable
                       # from the aim error alone - sweep gz before blaming the aim.
                       # (Was -75.0 before 2026-07-19 too, so this rig seems to sit near -75.)
GRASP_Z_DRIFT = 15.0   # observed spread between measurements; budget for it

# The same floor, felt with the jaws OPEN. Closed jaws reach 20 mm LOWER than open ones,
# because closing swings the fingertips down and in. I did not model this at all, and
# without it every grasp would have driven the closing jaws hard into the floor - which
# is exactly what happened: the arm is strong enough that instead of stalling it LIFTED
# THE WHOLE CAR off the ground. (So "servo failed to reach its target" is NOT a usable
# contact detector here.)
OPEN_JAW_FLOOR_Z = -110.0

# What kin.py *thought* the floor was. It is ~25 mm optimistic - the arm sags under its
# own weight, exactly as the earlier calibration residuals implied (6 px of error high up,
# 18 px down near the floor). Trust the measured numbers above, never the model's zero.
MODEL_PREDICTED_FLOOR_Z = -135.0

MIN_FLOOR_RADIUS = 60.0


def max_floor_radius():
    """Furthest the claw reaches at GRASP_Z. DERIVED - never hardcode it.

    It was hardcoded once, at 203 mm (computed back when the floor was believed to be at
    z=-135). When the measured floor moved to -75 the constant stayed put, and the servo
    loop began refusing perfectly reachable objects, 46 mm short of the truth. Anything
    that depends on GRASP_Z has to be recomputed when GRASP_Z changes - so compute it."""
    import kin
    return kin.max_reach(GRASP_Z)


# --- which way the arm is pointing, relative to the CAR ---

# Servo 6 = 470 aims the claw straight ahead of the vehicle. Fixed by the user on
# 2026-07-12; it is also arm.py's HOME_POSE base value. Note the servo's own centre (500)
# is NOT forward - it is just the middle of its travel, and means nothing physically.
BASE_FORWARD = 470

# Which way is which (established live - do not re-derive it by guessing):
#     servo 6 UP   -> claw swings LEFT
#     servo 6 DOWN -> claw swings RIGHT
# ~4 units per degree, so a sideways shift of D mm at reach R is 4*degrees(D/R) units.
#
# Backlash: the base consistently stops ~3 units short of the command (ask for 470, get
# 468; ask for 450, get 453) - about 0.75 deg, i.e. ~1.5 mm at a 108 mm reach. Harmless
# for grasping, but don't mistake it for a fault.
BASE_UNITS_PER_DEG = 4.0
BASE_BACKLASH_UNITS = 3


# --- the wrist camera ---

# The camera and the jaws are both bolted to the wrist, so THE CLAW NEVER MOVES IN THE
# IMAGE. Measured across five very different arm poses (base rotated, shoulder raised,
# different reach): the claw's pixel varied by 0.4 px.
#
# This collapses the whole grasping problem. With the old fixed camera I needed a 3D
# camera model, a homography, a calibrated zone, parallax corrections and a live search
# for the claw in every frame. Now: the jaws close at a FIXED PIXEL, and grasping means
# "manoeuvre the object onto that pixel, then close". No camera model, no zone - it works
# anywhere the arm can reach.
CLAW_IS_FIXED_IN_IMAGE = True

# ...BUT THE VALUE BELOW GOES STALE, AND A STALE ONE FAILS SILENTLY. "Fixed in the image"
# holds for a given camera mounting; it does NOT survive a remount. Measured live twice on
# 2026-07-26 and it was nowhere near this constant:
#     (325, 342) at base=480 R=165 z=GRASP_Z+38   (USB plug pose)
#     (392, 238) at base=326 R=165 z=GRASP_Z+60   (blue bar pose)
#
# REAL CHARGER USB (not the practice dummy) - confirmed grasp pose, 2026-07-28:
#     base=478, R=165, z=GRASP_Z+33 to close on it; the object's base sits ~30mm off the
#     floor (user-measured). Insertion/removal at this mount is VERTICAL (straight z, at
#     this same x,y) - not by extending reach. Use pick_eye.goto_vertical() for the
#     up/down leg once goto() has set the pitch here, or the wrist quietly leans forward
#     on every lift/insert and needs a manual few-mm correction back (see
#     [[real-charger-usb-session-2026-07-28]]).
# i.e. ~220 px right and ~90 px down of (170, 146). Since the aim loop drives the object ONTO
# this pixel, a stale value steers the object CONSISTENTLY OFF TO ONE SIDE of the real jaws -
# the object lands under one claw and the grasp closes beside it. The failure looks like bad
# luck, not bad calibration: the aim loop reports beautiful convergence (7-17 px) every time,
# because it is converging perfectly onto the wrong point. Four straight grasps failed this
# way before the user spotted it from the side ("брус попадает под левую клешню все время").
# Re-measuring and aiming at the live value grasped it first try (wiggle 9 px).
#
# SO: MEASURE IT LIVE at the start of a session, don't trust this number. One extra move:
#     arm.py step "1:700" ref.jpg 900      # close the empty jaws, grab a frame
#     # find the 2 red jaw markers (HSV ~[0-15]&[165-180], S>60, V>50, area>150);
#     # the midpoint of the two centroids IS the closing point for this pose.
# The residual pose-to-pose variation is small; the mounting-to-mounting drift is not.
GRASP_PIXEL = (170.0, 146.0)   # midpoint of the two jaw markers with the jaws CLOSED

# Blob-detection constants are VIEW-DEPENDENT and were all silently wrong after the camera
# moved to the wrist - each one threw away the very markers it was meant to find:
#   - the markers now cover ~2500-3300 px, not ~50 (they are inches from the lens)
#   - the camera looks out from BETWEEN the jaws, so they land on opposite sides of the
#     frame, ~314 px apart, not ~25
#   - with the jaws wide open, one of them swings out of frame entirely
# They live in calib.py; this note exists so the next surprise is not a surprise.
