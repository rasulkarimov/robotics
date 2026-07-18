#!/usr/bin/env python3
"""Forward/inverse kinematics for the Hiwonder xArm 1S.

WHY THIS EXISTS
Camera-guided grasping kept failing because I had no idea where the claw actually was
in space, or whether a target was even reachable. On 2026-07-11 an object got nudged
~5cm out of reach and I blindly swept joint values for half an hour without ever being
able to say "that point is outside my working envelope". This module fixes that: it
answers "where is the claw" (fk) and "what joints put the claw at (x,y,z)" (ik), and
`reachable()` says yes/no before you move anything.

WHERE THE NUMBERS COME FROM (none of this is guessed)
- Link lengths: the community ROS URDF for this exact arm,
  github.com/diestra-ai/xArm_Lewansoul_ROS -> xarm_description/urdf/xarm.urdf
  (the URDF's joint chain also CORRECTED my joint map: the chain runs
   6=base -> 5=shoulder -> 4=elbow -> 3=wrist_pitch -> 2=wrist_rotate -> 1=gripper.
   I originally had 3 and 5 swapped; confirmed by moving each alone on camera -
   servo 5 swings the whole arm, servo 3 only tilts the end.)
- Angle mapping: servo units 0..1000 span -125..+125 deg (0.25 deg/unit), and
  position 500 = 0 deg. Verified empirically: commanding 3,4,5 = 500 stands the arm
  perfectly VERTICAL, which is exactly the URDF zero pose. So angle = (pos-500)*0.25.
- Joint rotation signs and the gripper length: solved numerically, not measured. The
  gripper is NOT in the URDF, so I took poses where the claw was provably touching the
  floor (z=0, from the hand-taught grab) and searched all 8 sign combinations for one
  that yields a physically plausible gripper length and reach. Exactly ONE did:
  signs (shoulder -1, elbow +1, wrist -1) with Lg = 63.6mm, reach 106.6mm. A unique
  plausible solution is strong evidence it's correct.

CAVEAT: the arm's base plate is assumed to sit on the same plane as the object (both
on the floor). If the arm gets mounted on the car chassis, add the chassis height to
BASE_HEIGHT or every z will be wrong.
"""
import math

# --- geometry, millimetres (from the URDF) ---
BASE_HEIGHT = 36.03 + 31.95   # base plate -> shoulder(5) axis
L1 = 97.65                    # shoulder(5) -> elbow(4)
L2 = 98.25                    # elbow(4)    -> wrist_pitch(3)
L3 = 53.06                    # wrist_pitch(3) -> wrist_rotate(2) axis
LG = 63.6                     # wrist_rotate(2) axis -> grasp point (solved, see above)
L3G = L3 + LG                 # last link, wrist_pitch axis -> grasp point

# --- servo <-> angle ---
UNITS_PER_DEG = 4.0           # 1000 units / 250 deg
CENTER = 500                  # servo 500 = 0 deg = arm straight up

# rotation sign of each pitch joint, solved numerically (see docstring).
# SIGN[6] (base) only mirrors the y axis; +1 means increasing servo 6 sweeps toward +y.
SIGN = {6: +1, 5: -1, 4: +1, 3: -1}

# servo travel limits (0..1000); the URDF also caps the joints well short of the
# servo's full electrical range, so don't command beyond these
SERVO_MIN, SERVO_MAX = 100, 900


def s2a(servo, joint):
    """servo units -> joint angle in radians (0 = straight up)"""
    return math.radians(SIGN[joint] * (servo - CENTER) / UNITS_PER_DEG)


def a2s(angle_rad, joint):
    """joint angle (radians) -> servo units"""
    return CENTER + SIGN[joint] * math.degrees(angle_rad) * UNITS_PER_DEG


def fk(s5, s4, s3, s6=CENTER):
    """Forward kinematics. Returns (x, y, z) of the GRASP POINT in mm, origin at the
    centre of the base plate, z up. s6 (base rotation) sweeps x/y."""
    f1 = s2a(s5, 5)              # shoulder, from vertical
    f2 = f1 + s2a(s4, 4)         # + elbow
    f3 = f2 + s2a(s3, 3)         # + wrist pitch
    r = L1 * math.sin(f1) + L2 * math.sin(f2) + L3G * math.sin(f3)
    z = BASE_HEIGHT + L1 * math.cos(f1) + L2 * math.cos(f2) + L3G * math.cos(f3)
    base = s2a(s6, 6)
    return r * math.cos(base), r * math.sin(base), z


def ik(x, y, z, pitch_deg=180.0):
    """Inverse kinematics. Put the grasp point at (x,y,z) mm with the claw pointing at
    `pitch_deg` from vertical (180 = straight DOWN at the floor, which is what you want
    for picking things up off the ground).

    Returns dict {6: base, 5: shoulder, 4: elbow, 3: wrist} in servo units, or None if
    the point is out of reach. Standard 3-link planar solution: fix the tool angle,
    subtract the last link to get the wrist centre, then 2-link law-of-cosines."""
    base = math.atan2(y, x)
    r = math.hypot(x, y)
    f3 = math.radians(pitch_deg)

    # wrist-pitch joint position, after backing off the last link along the tool axis
    rw = r - L3G * math.sin(f3)
    zw = z - BASE_HEIGHT - L3G * math.cos(f3)

    d = math.hypot(rw, zw)
    if d > L1 + L2 or d < abs(L1 - L2):
        return None                                   # outside the 2-link annulus

    # elbow via law of cosines
    cos_e = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    cos_e = max(-1.0, min(1.0, cos_e))
    elbow = math.acos(cos_e)                          # elbow-down solution

    f1 = math.atan2(rw, zw) - math.atan2(L2 * math.sin(elbow), L1 + L2 * math.cos(elbow))
    f2 = f1 + elbow
    wrist = f3 - f2                                   # tool angle constraint

    out = {6: a2s(base, 6), 5: a2s(f1, 5), 4: a2s(elbow, 4), 3: a2s(wrist, 3)}
    for j, v in out.items():
        if not (SERVO_MIN <= v <= SERVO_MAX):
            return None                               # joint can't get there
    return {j: int(round(v)) for j, v in out.items()}


def ik_search(x, y, z, pitch_lo=150.0, pitch_hi=225.0, prefer=195.0):
    """IK that SEARCHES the approach angle instead of demanding an exact one.

    A fixed pitch is over-constrained and rejects perfectly reachable points - the
    hand-taught grasp turned out to approach at 195 deg, not the 180 deg ("straight
    down") I'd assumed, and demanding 180 made IK declare that very pose unreachable.
    Hiwonder's own reference API takes a pitch RANGE for the same reason.

    Searches outward from `prefer` and returns the feasible solution closest to it."""
    for delta in range(0, int(max(pitch_hi - prefer, prefer - pitch_lo)) + 1):
        for p in ({prefer + delta, prefer - delta} if delta else {prefer}):
            if pitch_lo <= p <= pitch_hi:
                sol = ik(x, y, z, p)
                if sol:
                    sol["pitch"] = round(p, 1)
                    return sol
    return None


def reachable(x, y, z):
    return ik_search(x, y, z) is not None


def max_reach(z=0.0):
    """Furthest radius the grasp point can reach at height z, over all approach angles.
    This is the number that would have saved half an hour of blind joint-sweeping."""
    best = 0.0
    for mm in range(20, 400):
        if reachable(float(mm), 0.0, z):
            best = float(mm)
    return best


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 4 and sys.argv[1] == "fk":
        s5, s4, s3 = (int(v) for v in sys.argv[2:5])
        s6 = int(sys.argv[5]) if len(sys.argv) > 5 else CENTER
        x, y, z = fk(s5, s4, s3, s6)
        print(f"grasp point: x={x:.1f} y={y:.1f} z={z:.1f} mm  (r={math.hypot(x,y):.1f})")
    elif len(sys.argv) >= 4 and sys.argv[1] == "ik":
        x, y, z = (float(v) for v in sys.argv[2:5])
        pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 180.0
        sol = ik(x, y, z, pitch)
        print(sol if sol else f"UNREACHABLE ({x},{y},{z}); max reach at z={z}: {max_reach(z):.0f}mm")
    elif len(sys.argv) >= 2 and sys.argv[1] == "envelope":
        for z in (0, 20, 50, 100):
            print(f"z={z:3d}mm -> max radius {max_reach(float(z)):.0f}mm")
    else:
        print(__doc__)
        print("usage: kin.py fk S5 S4 S3 [S6] | kin.py ik X Y Z [pitch] | kin.py envelope")
