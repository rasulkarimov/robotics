#!/usr/bin/env python3
"""Two-stage grasp: aim from HIGH UP first, then descend and only fine-tune if needed.

WHY TWO STAGES: aiming at grasp height alone kept losing the target - down there the camera
is centimetres off the floor, the field of view is tiny, and an object that isn't already
well aligned simply falls out of frame, leaving nothing to steer by. Aim high, where the
view is wide.

WHY THE BOTTOM PASS IS TIMID: close up, the object fills much of the frame and touches its
edge. A clipped blob's CENTROID jumps around, so the Jacobian measured from it lies, and a
"correction" can make things far worse - one run went 48 px -> 269 px -> target lost. So at
the bottom we only correct if the alignment is actually poor, and we never let a correction
undo a good approach: if it starts diverging, we grasp from where the top pass put us.

This works only because the claw is rigid to the camera: GRASP_PIXEL is the correct target
at ANY height, so one aim point serves both passes.
"""
import math, subprocess, time
import numpy as np
import kin, rig
import pick_eye as pe

GOOD_ENOUGH_PX = 60.0     # jaws are wide; closer than this and correcting adds only risk


def pose(base, R, z, ms=1600):
    a = kin.s2a(base, 6); x, y = R*math.cos(a), R*math.sin(a)
    s = kin.ik_search(x, y, z, pitch_lo=pe.FIXED_PITCH-pe.PITCH_BAND,
                      pitch_hi=pe.FIXED_PITCH+pe.PITCH_BAND, prefer=pe.FIXED_PITCH)
    if not s: return None
    s[6] = base
    subprocess.run(["/home/astra/robotics/arm","step", ",".join(f"{j}:{s[j]}" for j in (6,5,4,3)),
                    "/tmp/x.jpg", str(ms)], stdout=subprocess.DEVNULL)
    return x, y


def err_now():
    p = pe.see()
    if p is None:
        return None
    return math.dist(p, rig.GRASP_PIXEL)


def grab(base, R):
    HIGH = rig.GRASP_Z + 60.0
    if pose(base, R, HIGH, 1800) is None:
        print("   поза подхода недостижима"); return False
    pe.arm_step(f"1:{pe.OPEN}", 700); time.sleep(0.5)
    if pe.see() is None:
        print("   ПРЕДМЕТ НЕ ВИЖУ"); return False

    a = kin.s2a(base, 6); x, y = R*math.cos(a), R*math.sin(a)

    r = pe.servo(x, y, HIGH, iters=6, tol_px=35.0, label="сверху")
    if r is None:
        return False
    x, y = r

    if not pe.goto(x, y, rig.GRASP_Z, 1400):
        print("   спуск недостижим"); return False

    e = err_now()
    if e is None:
        print("   внизу предмет не виден — не смыкаю"); return False
    print(f"   внизу: ошибка {e:.0f} px")
    if e > GOOD_ENOUGH_PX:
        r = pe.servo(x, y, rig.GRASP_Z, iters=3, tol_px=GOOD_ENOUGH_PX, label="доводка")
        if r is not None:
            x, y = r
        else:
            # correction failed - fall back on the (good) alignment from the top pass
            print("   доводка не удалась — беру по наведению сверху")
            pe.goto(x, y, rig.GRASP_Z, 900)
            if err_now() is None:
                print("   предмет пропал — не смыкаю"); return False

    pe.arm_step(f"1:{pe.CLAMP}", 900)
    pe.goto(x, y, rig.GRASP_Z + 90, 1500)
    return True


def put(base, R):
    pose(base, R, rig.GRASP_Z + 90, 2000)
    pose(base, R, rig.GRASP_Z, 1600)
    pe.arm_step("1:156", 900)
    pose(base, R, rig.GRASP_Z + 90, 1500)
