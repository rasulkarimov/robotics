#!/usr/bin/env python3
"""grasp.py -- the grasp expressed as an explicit act -> verify -> retry/escalate loop.

This does NOT reinvent grasping. tanggrab.py already holds all the hard-won motion
(wrist-across-the-short-axis, reach pull-in, clean aiming, the held-shift test). What
this adds is the *loop discipline* from looplib: one grab is one attempt, the held-test
is the verifiable signal, a miss lowers the grasp z and retries, and exhausting the
retries escalates to a human instead of silently walking off with empty jaws.

tanggrab.drill() already did exactly this by hand, per-rep. grasp_until_held() is that
pattern factored out so a single grab, a place, and a reps-drill all share ONE loop and
one escalation path -- and so it can be unit-tested without the robot (the real grab is
injected, see main()).  Test:  python3 grasp.py --selftest

Real run (reuses all of tanggrab's motion + auto-locate):
  python3 grasp.py --base auto --gz -30 --place
  python3 grasp.py --reps 3 --drill 460:140
"""
import argparse
import sys

import looplib

HELD_TOL_PX = 18.0        # held_shift below this = the object really moved with the jaws
DROP_MM = 6.0             # lower the grasp z by this on each missed attempt (from drill())
MAX_ATTEMPTS = 3          # tries before we escalate (drill() used 2; 3 is a touch safer)


def grasp_until_held(base, R, gz0, outdir, grab_fn, *,
                     max_attempts=MAX_ATTEMPTS, drop_mm=DROP_MM, held_tol=HELD_TOL_PX,
                     log=print):
    """Grab at (base, R), verify the held-test, and on a miss lower z and retry.

    grab_fn(base, R, gz, outdir) -> dict|falsey : the actual grab. Matches
        tanggrab.tanggrab's contract: returns a dict with 'held' (bool) and 'shift'
        (px, or None), or a falsey value if the object was lost before the held-test.
    Returns a looplib.LoopResult; .last.result is the winning (or final) grab dict.
    """
    plan = {"gz": float(gz0)}

    def act(_n):
        res = grab_fn(base, R, plan["gz"], outdir)
        if isinstance(res, dict):
            res.setdefault("gz", plan["gz"])   # tanggrab omits gz; record which z won
        return res

    def verify(res):
        if not res:                                   # object lost mid-grab
            return False, None
        shift = res.get("shift")
        held = bool(res.get("held")) and (shift is None or shift < held_tol)
        return held, shift

    def adapt(_n, _last):
        plan["gz"] -= drop_mm
        log(f"  [grasp] not held; lowering grasp z -> {plan['gz']:.0f} mm")

    def escalate(last):
        s = (last.result or {}).get("shift") if last and last.result else None
        log(f"  [grasp] could not secure the object after {max_attempts} tries "
            f"(last held_shift={s}). Reposition it / check the jaws, then retry.")

    return looplib.run_until(act, verify, max_attempts=max_attempts, adapt=adapt,
                             on_escalate=escalate, label="grasp", log=log)


def _drill(base, drill_spot, gz0, reps, outdir, grab_fn, place_fn, s2a, log=print):
    """reps grab+place cycles at a FIXED spot, each grab a full verify-retry loop.
    Stops (escalates) the whole drill the first time a grab can't be secured -- same
    policy as tanggrab.drill(), but the per-rep retry now runs through looplib."""
    import math
    db, dr = int(drill_spot.split(":")[0]), float(drill_spot.split(":")[1])
    a = s2a(db, 6)
    dx, dy = dr * math.cos(a), dr * math.sin(a)
    results = []
    for rep in range(1, reps + 1):
        log(f"\n=== REP {rep}/{reps} ===")
        lr = grasp_until_held(base, 130.0, gz0, outdir, grab_fn, log=log)  # tanggrab always grabs at R=130
        results.append(lr)
        if not lr.ok:
            log(f"  [drill] rep {rep} could not be secured -- stopping the drill.")
            break
        gz_used = lr.last.result.get("gz", gz0) if lr.last.result else gz0
        place_fn(dx, dy, gz_used)
        base = db
    log("\n=== DRILL SUMMARY ===")
    for i, lr in enumerate(results, 1):
        s = lr.last.signal if lr.last else None
        log(f"rep{i}: held={lr.ok} shift={s} attempts={lr.n_attempts}")
    return results


def main(argv=None):
    ap = argparse.ArgumentParser(description="grasp with an explicit verify-retry loop")
    ap.add_argument("--base", default="auto", help="object base angle, or 'auto' to locate")
    ap.add_argument("--gz", type=float, default=-30.0, help="initial grasp close height (mm)")
    ap.add_argument("--place", action="store_true", help="set the object down after a held grab")
    ap.add_argument("--reset-to", default="445:128", help="base:R to place a single grab at")
    ap.add_argument("--reps", type=int, default=1, help="run a grab+place DRILL this many times")
    ap.add_argument("--drill", default="460:140", help="base:R fixed spot the drill uses")
    ap.add_argument("--max-attempts", type=int, default=MAX_ATTEMPTS)
    ap.add_argument("--outdir", default="/home/astra/robotics/orbit_out")
    args = ap.parse_args(argv)

    # Hardware/vision imports happen HERE, lazily, so `import grasp` (and --selftest)
    # never pull in cv2 or the arm modules.
    import math
    import tanggrab
    import orbit
    import kin

    if args.base == "auto":
        loc = orbit.locate()
        if loc is None:
            print("  [grasp] no blue object found"); return 1
        base, R = loc
    else:
        base, R = int(args.base), 130.0

    if args.reps > 1:
        _drill(base, args.drill, args.gz, args.reps, args.outdir,
               tanggrab.tanggrab, tanggrab.place, kin.s2a)
        return 0

    lr = grasp_until_held(base, R, args.gz, args.outdir, tanggrab.tanggrab,
                          max_attempts=args.max_attempts)
    if lr.ok and args.place:
        rb, rr = args.reset_to.split(":")
        a = kin.s2a(int(rb), 6)
        gz_used = lr.last.result.get("gz", args.gz)
        tanggrab.place(float(rr) * math.cos(a), float(rr) * math.sin(a), gz_used)
    print("RESULT:", (lr.last.result if lr.last else None), "| held:", lr.ok)
    return 0 if lr.ok else 2


# --------------------------------------------------------------------------- #
# Self-test: a fake grab that only "holds" once z has dropped enough. No hardware.
#   python3 grasp.py --selftest
# --------------------------------------------------------------------------- #
def _selftest():
    failures = 0

    def check(name, cond):
        nonlocal failures
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            failures += 1

    calls = []

    def fake_grab_succeeds_third(base, R, gz, outdir):
        calls.append(gz)
        held = gz <= -42            # needs two 6mm drops from -30 to reach -42
        return {"held": held, "shift": 3.0 if held else 40.0, "gz": gz,
                "base": base, "R": R}

    lr = grasp_until_held(455, 130.0, -30.0, "/tmp", fake_grab_succeeds_third,
                          max_attempts=3)
    check("succeeds on the 3rd attempt", lr.ok and lr.n_attempts == 3)
    check("z was lowered -30 -> -36 -> -42", calls == [-30.0, -36.0, -42.0])
    check("winning shift is the held one", lr.last.signal == 3.0)

    # object lost every time -> escalates.
    lr2 = grasp_until_held(455, 130.0, -30.0, "/tmp",
                           lambda *a: None, max_attempts=2)
    check("lost object every try -> escalate", (not lr2.ok) and lr2.escalated)

    # held flag set but shift too large -> NOT held (guards a false positive).
    lr3 = grasp_until_held(455, 130.0, -30.0, "/tmp",
                           lambda b, R, gz, o: {"held": True, "shift": 99.0},
                           max_attempts=1)
    check("large held_shift overrides held=True", not lr3.ok)

    print(f"\nselftest: {'ALL PASS' if failures == 0 else f'{failures} FAILURE(S)'}")
    return 1 if failures else 0


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
    sys.exit(main())
