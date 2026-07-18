#!/usr/bin/env python3
"""navloop.py -- a forward drive expressed as act -> verify -> correct/escalate.

car.drive_mm is honest about being open-loop ("фактическое расстояние не проверено"):
it fires a timed pulse and hopes. On this chassis a "straight" leg actually veers,
because the arm-mount body has been fouling the steer wheel (see memory
[[turn-arc-mechanics]]; the user is cutting wheel-wells to fix it). So a drive with no
check silently corrupts the dead-reckoned pose.

This wraps the drive in looplib's loop and wires it to a *verifiable signal*: dxyaw
measures the heading change from central-feature disparity (+ = turned LEFT/CCW). The
leg counts as straight only if |drift| <= tolerance; otherwise the loop steers the
OPPOSITE way for a short corrective nudge and re-measures, escalating if it can't get
the body back on heading. The net measured drift is returned so the caller can keep the
world pose honest even when a residual remains.

Hardware + vision are injected (car.*, dxyaw.analyze), so the control logic self-tests
with mocks and no robot / no cv2:  python3 navloop.py --selftest

CALIBRATION: the correction gains below are first guesses. Re-tune them on the real car
AFTER the wheel-well fix, with the user watching -- do not trust them blind.
"""
import argparse
import os
import sys
import time

import looplib

YAW_TOL_DEG = 4.0     # within this, a leg counts as straight (dxyaw centre-band noise ~1-2 deg)
CORRECT_STEER = 35    # steer angle for a counter-steer correction leg (car.steer clamps 10..60)
CORRECT_MM = 40       # short corrective nudge distance (mm) per correction leg
MAX_ATTEMPTS = 3      # 1 main leg + up to 2 corrections before escalating
SETTLE = 0.4          # let the frame settle before snapping (motion blur / rolling shutter)


def drive_straight_verified(mm, drive_fn, snap_fn, yaw_fn, steer_fn, *,
                            frame_before, frame_after,
                            yaw_tol=YAW_TOL_DEG, max_attempts=MAX_ATTEMPTS,
                            correct_steer=CORRECT_STEER, correct_mm=CORRECT_MM,
                            settle=SETTLE, log=print):
    """Drive forward `mm`, verify straightness with dxyaw, counter-steer on drift.

    Injected callables (so this is testable without the robot):
      drive_fn(direction, mm)      -> drive a leg (car.drive_mm)
      snap_fn(path)                -> capture a frame to `path` (car.snapshot)
      yaw_fn(before, after) -> deg -> heading change, + = turned LEFT/CCW (dxyaw.analyze)
      steer_fn(direction, angle)   -> set steering ('center'/'left'/'right') (car.steer)

    Returns (LoopResult, net_yaw_deg). net_yaw_deg is the cumulative measured drift
    (~0 on success; the uncorrected residual if it escalated) -- feed it back into the
    world pose so the map stays honest.
    """
    plan = {"cum": 0.0, "steer": None}

    def act(n):
        snap_fn(frame_before)
        if n == 1:
            steer_fn("center", 90)
            drive_fn("forward", mm)              # the real leg
        else:
            d, ang = plan["steer"]
            steer_fn(d, ang)
            drive_fn("forward", correct_mm)      # short counter-steer correction
            steer_fn("center", 90)               # re-centre after correcting
        if settle:
            time.sleep(settle)
        snap_fn(frame_after)
        leg_yaw = float(yaw_fn(frame_before, frame_after))
        plan["cum"] += leg_yaw
        return {"attempt": n, "leg_yaw": round(leg_yaw, 2), "cum_yaw": round(plan["cum"], 2)}

    def verify(res):
        return abs(res["cum_yaw"]) <= yaw_tol, res["cum_yaw"]

    def adapt(_n, _last):
        # Cancel the accumulated drift: turned LEFT (+) -> steer RIGHT, and vice-versa.
        cum = plan["cum"]
        d = "right" if cum > 0 else "left"
        plan["steer"] = (d, int(max(10, min(60, correct_steer))))
        log(f"  [nav] drift {cum:+.1f} deg -> correction leg: steer {d} {plan['steer'][1]}, "
            f"nudge {correct_mm} mm")

    def escalate(last):
        cum = last.result["cum_yaw"] if last and last.result else plan["cum"]
        log(f"  [nav] could not hold heading (residual {cum:+.1f} deg after {max_attempts} "
            f"legs). Recording the residual so the pose stays honest; check the steering.")

    lr = looplib.run_until(act, verify, max_attempts=max_attempts, adapt=adapt,
                           on_escalate=escalate, label="nav", log=log)
    return lr, plan["cum"]


def main(argv=None):
    ap = argparse.ArgumentParser(description="drive forward with a verified-straight loop")
    ap.add_argument("mm", type=float, help="forward distance in mm")
    ap.add_argument("--yaw-tol", type=float, default=YAW_TOL_DEG)
    ap.add_argument("--max-attempts", type=int, default=MAX_ATTEMPTS)
    ap.add_argument("--correct-steer", type=int, default=CORRECT_STEER)
    ap.add_argument("--correct-mm", type=float, default=CORRECT_MM)
    ap.add_argument("--update-world", action="store_true",
                    help="advance the nav world pose by mm and set heading to the residual drift")
    ap.add_argument("--outdir", default="/home/astra/tools/nav_state")
    args = ap.parse_args(argv)

    # Lazy: dxyaw pulls in cv2, car talks to the server. Neither is needed to import
    # this module or run --selftest.
    import car
    import dxyaw

    os.makedirs(args.outdir, exist_ok=True)
    fb = os.path.join(args.outdir, "_leg_before.jpg")
    fa = os.path.join(args.outdir, "_leg_after.jpg")

    lr, net_yaw = drive_straight_verified(
        args.mm, car.drive_mm, car.snapshot, dxyaw.analyze, car.steer,
        frame_before=fb, frame_after=fa,
        yaw_tol=args.yaw_tol, max_attempts=args.max_attempts,
        correct_steer=args.correct_steer, correct_mm=args.correct_mm,
    )
    print(f"\nRESULT: straight={lr.ok} net_drift={net_yaw:+.1f} deg attempts={lr.n_attempts}")

    if args.update_world:
        import math
        import nav
        w = nav.load_world()
        nav._apply_motion(w, float(args.mm), +1)          # advance along current heading
        w["pose"]["theta_deg"] = round(w["pose"]["theta_deg"] + net_yaw, 1)  # fold in residual
        nav.save_world(w)
        nav.cmd_pose(None)
    return 0 if lr.ok else 2


# --------------------------------------------------------------------------- #
# Self-test: a fake car whose "drift" shrinks as corrections are applied. The mock
# yaw_fn reports drift-per-leg with the right SIGN so adapt() steers the right way.
#   python3 navloop.py --selftest
# --------------------------------------------------------------------------- #
def _selftest():
    failures = 0

    def check(name, cond):
        nonlocal failures
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            failures += 1

    class FakeCar:
        """Main leg veers +12 deg (left). Each RIGHT correction removes ~7 deg; a LEFT
        correction would ADD (wrong way). Records the steering it was told to use."""
        def __init__(self):
            self.pending = 0.0      # drift this next 'after' frame will reveal
            self.steer_log = []
            self.legs = 0
        def drive(self, direction, mm):
            self.legs += 1
        def snap(self, path):
            pass
        def steer(self, direction, angle):
            self.steer_log.append((direction, angle))
        def yaw(self, before, after):
            # first leg: +12; corrections: -7 if we steered right, +7 if left (wrong way)
            if self.legs == 1:
                return 12.0
            last_dir = [d for d, a in self.steer_log if d in ("left", "right")]
            return -7.0 if (last_dir and last_dir[-1] == "right") else +7.0

    fc = FakeCar()
    lr, net = drive_straight_verified(
        300, fc.drive, fc.snap, fc.yaw, fc.steer,
        frame_before="/tmp/b.jpg", frame_after="/tmp/a.jpg", settle=0.0,
    )
    # +12 -> steer right -> +12-7=+5 (>4) -> steer right -> +5-7=-2 (<=4) : ok on attempt 3
    check("converges within tolerance", lr.ok and lr.n_attempts == 3)
    check("net drift within tol", abs(net) <= YAW_TOL_DEG)
    check("corrected by steering RIGHT (cancels a left drift)",
          ("right", CORRECT_STEER) in fc.steer_log and ("left", CORRECT_STEER) not in fc.steer_log)

    # A leg that stays straight from the start: one attempt, no correction.
    straight = FakeCar()
    straight.yaw = lambda b, a: 0.5
    lr2, net2 = drive_straight_verified(
        200, straight.drive, straight.snap, straight.yaw, straight.steer,
        frame_before="/tmp/b.jpg", frame_after="/tmp/a.jpg", settle=0.0,
    )
    check("already-straight leg -> 1 attempt, no correction", lr2.ok and lr2.n_attempts == 1)

    # Drift the loop can't fix (correction never helps) -> escalate with the residual.
    stuck = FakeCar()
    stuck.yaw = lambda b, a: 15.0     # always +15, corrections never register
    lr3, net3 = drive_straight_verified(
        300, stuck.drive, stuck.snap, stuck.yaw, stuck.steer,
        frame_before="/tmp/b.jpg", frame_after="/tmp/a.jpg", settle=0.0, max_attempts=3,
    )
    check("unfixable drift -> escalate", (not lr3.ok) and lr3.escalated)
    check("residual drift reported (non-zero)", abs(net3) > YAW_TOL_DEG)

    print(f"\nselftest: {'ALL PASS' if failures == 0 else f'{failures} FAILURE(S)'}")
    return 1 if failures else 0


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
    sys.exit(main())
