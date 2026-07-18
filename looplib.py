#!/usr/bin/env python3
"""looplib.py -- the one small primitive behind every reliable robot action here.

Loop-engineering, in one line: *an action is only as good as the verifiable signal
you wire it to.* So every physical action on this robot should go through the same
shape instead of "fire the servo and hope":

        ACT  ->  VERIFY  ->  (retry with adaptation | escalate to human)

This module is that shape, written once, reusable. It grew out of the pattern that
tanggrab.drill() already discovered by hand: grab -> held-test -> if not held, drop
6mm and retry -> if still not, stop. That is exactly run_until() below, generalised.

Callers on this robot:
  grasp  (grasp.py)   act = centre+descend+clamp+lift, verify = held-shift test,
                      adapt = lower grasp z, escalate = ask a human to reposition.
  nav    (navloop.py) act = drive a leg, verify = yaw-drift / arrival check,
                      adapt = counter-steer the residual, escalate = stop & report.

Nothing here touches hardware -- the caller passes in `act` and `verify` callables,
so the loop logic is fully testable with mocks:  python3 looplib.py --selftest
"""
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Tuple


@dataclass
class Attempt:
    """One pass through the loop: what the action returned and how it verified."""
    n: int                 # 1-based attempt number
    result: Any            # whatever act() returned (often a dict)
    signal: Any            # verify()'s detail (e.g. held_shift in px), for logging
    ok: bool               # did the verifiable signal say success?


@dataclass
class LoopResult:
    """The outcome of a run_until(): success/failure plus the full attempt trail."""
    ok: bool
    attempts: List[Attempt] = field(default_factory=list)
    escalated: bool = False
    label: str = "action"

    @property
    def n_attempts(self) -> int:
        return len(self.attempts)

    @property
    def last(self) -> Optional[Attempt]:
        return self.attempts[-1] if self.attempts else None

    def __bool__(self) -> bool:      # so `if run_until(...):` reads naturally
        return self.ok


def _split_verify(v: Any) -> Tuple[bool, Any]:
    """verify() may return a plain bool, or (ok, detail). Normalise to (ok, detail)."""
    if isinstance(v, tuple) and len(v) == 2 and isinstance(v[0], (bool, int)):
        return bool(v[0]), v[1]
    return bool(v), None


def run_until(
    act: Callable[[int], Any],
    verify: Callable[[Any], Any],
    *,
    max_attempts: int = 3,
    adapt: Optional[Callable[[int, Attempt], None]] = None,
    on_escalate: Optional[Callable[[Attempt], None]] = None,
    label: str = "action",
    settle: float = 0.0,
    log: Callable[[str], None] = print,
) -> LoopResult:
    """Do `act`, check `verify`, retry with `adapt`, else `escalate`. The whole loop.

    Args:
      act(attempt_n) -> result : perform the action; receives the 1-based attempt
          number. Bind base/R/gz via a closure and read mutable plan state so
          `adapt` can change it between tries (that is how grasp lowers z).
      verify(result) -> ok | (ok, detail) : the *verifiable signal*. Return a bool,
          or (bool, detail) where detail is logged (e.g. the held-shift in px).
      max_attempts : hard cap on tries before escalating (>=1).
      adapt(attempt_n, last_attempt) -> None : called after a FAILED attempt,
          before the next act, to change the plan (lower z, counter-steer, ...).
          Omit for a plain retry-identically loop.
      on_escalate(last_attempt) -> None : called once if every attempt failed --
          this is the "hand it to a human / stop safely" hook. It does not raise;
          run_until always returns a LoopResult so the caller decides what to do.
      settle : optional seconds to sleep before each attempt (let motion settle).
      log : where progress lines go (default print; pass a no-op to silence).

    Returns a LoopResult (truthy iff it succeeded).
    """
    if max_attempts < 1:
        raise ValueError("max_attempts must be >= 1")
    res = LoopResult(ok=False, label=label)
    for n in range(1, max_attempts + 1):
        if adapt is not None and n > 1:
            adapt(n, res.last)               # change the plan before retrying
        if settle:
            time.sleep(settle)
        result = act(n)
        ok, detail = _split_verify(verify(result))
        attempt = Attempt(n=n, result=result, signal=detail, ok=ok)
        res.attempts.append(attempt)
        mark = "OK" if ok else "miss"
        log(f"  [{label}] attempt {n}/{max_attempts}: {mark}"
            + (f" (signal={detail})" if detail is not None else ""))
        if ok:
            res.ok = True
            return res
    # every attempt failed
    res.escalated = True
    log(f"  [{label}] ESCALATE after {res.n_attempts} attempts -- needs a human")
    if on_escalate is not None:
        on_escalate(res.last)
    return res


# --------------------------------------------------------------------------- #
# Self-test: exercises the loop logic with mocks. No hardware, no imports of the
# robot modules. Run:  python3 looplib.py --selftest
# --------------------------------------------------------------------------- #
def _selftest() -> int:
    failures = 0

    def check(name, cond):
        nonlocal failures
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            failures += 1

    # 1) first-try success: one attempt, ok, not escalated.
    r = run_until(lambda n: {"v": 1}, lambda res: True, label="first-ok")
    check("first-try success -> ok", r.ok and r.n_attempts == 1 and not r.escalated)

    # 2) succeeds on the 3rd try after adaptation (mirrors grasp lowering z).
    plan = {"z": 0}
    def act(n):
        return {"z": plan["z"]}
    def verify(res):
        # "held" only once z has been lowered twice (z == -12), with a detail value.
        held = res["z"] <= -12
        return held, abs(res["z"])          # detail = |z|, like a held-shift
    def adapt(n, last):
        plan["z"] -= 6                       # drop 6mm each miss, like drill()
    r = run_until(act, verify, max_attempts=4, adapt=adapt, label="grasp-sim")
    check("adaptive retry succeeds on attempt 3", r.ok and r.n_attempts == 3)
    check("adapt lowered z to -12", plan["z"] == -12)
    check("last signal recorded", r.last.signal == 12)

    # 3) never succeeds -> escalates, on_escalate fires exactly once.
    esc = {"count": 0}
    r = run_until(
        lambda n: n, lambda res: (False, "no-signal"),
        max_attempts=3, on_escalate=lambda last: esc.__setitem__("count", esc["count"] + 1),
        label="never",
    )
    check("exhausts attempts -> not ok", not r.ok and r.n_attempts == 3)
    check("escalated flag set", r.escalated)
    check("on_escalate called once", esc["count"] == 1)

    # 4) bare-bool verify (no detail) is accepted.
    r = run_until(lambda n: None, lambda res: True, label="bare-bool")
    check("bare bool verify works", r.ok and r.last.signal is None)

    # 5) __bool__ convenience.
    check("LoopResult is truthy when ok", bool(run_until(lambda n: 1, lambda r: True)))

    print(f"\nselftest: {'ALL PASS' if failures == 0 else f'{failures} FAILURE(S)'}")
    return 1 if failures else 0


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
    print(__doc__)
