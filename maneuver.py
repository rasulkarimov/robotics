#!/usr/bin/env python3
"""Batched sense+move with in-loop clearance gating (the "1+2" speed-up, 2026-07-18).

One invocation runs a whole maneuver (many pulses), snaps a frame per step, computes
yaw (dxyaw) + clearance (perceive) IN THE LOOP, self-aborts on a close obstacle, and
emits ONE montage + a compact summary — instead of round-tripping every pulse through
the LLM. No ultrasonic (dead) -> camera is the only sensor, so motion still waits on a
fresh frame, but the whole sense->decide->act cycle now runs on the Pi.

Camera contract: neck forward (servo6=500) + wrist level (servo5=514) so each snapshot
yields BOTH yaw and clearance with no tilt. `--level-cam` re-establishes that pose first.

Commands (all print a JSON-ish summary + write a montage PNG):
  maneuver.py sense OUT.png [--level-cam]
      one frame; clearance only.
  maneuver.py turn <left|right> N OUT.png [--dur 0.6] [--steer 60] [--min-clear 0.28] [--level-cam]
      N steered pulses; per pulse: pre-clearance gate, pulse, post frame -> yaw+clearance.
      Aborts before any pulse whose current centre clearance < min-clear (arc lunges fwd).
  maneuver.py forward MM OUT.png [--min-clear 0.30] [--level-cam]
      clearance-gated straight drive; caps distance when centre is only moderately clear.

Clearance centre free_frac heuristic (validated on real frames): >=0.55 open,
0.30-0.55 something at mid distance (short legs), <0.30 close obstacle (blocked).
"""
import sys
import os
import time
import json
import argparse
import io
import contextlib

sys.path.insert(0, "/home/astra/tools")
import car
import perceive
import dxyaw


def level_cam():
    """Neck forward + wrist level, so frames serve both yaw and clearance."""
    import subprocess
    for sid, pos in ((6, 500), (5, 514)):
        subprocess.run(["sudo", "/home/astra/tools/venv/bin/python3",
                        "/home/astra/robotics/arm.py", "move", str(sid), str(pos), "700"],
                       capture_output=True)
    time.sleep(0.4)


def yaw(fa, fb):
    with contextlib.redirect_stdout(io.StringIO()):
        return dxyaw.analyze(fa, fb)


def snap(path):
    car.snapshot(path)
    return perceive.clearance(path)


def _c(m):  # compact clearance dict
    b = m["bands"]
    return {"L": b["left"], "C": b["center"], "R": b["right"],
            "min": m["min"], "clear": m["clearest"]}


def do_sense(args):
    if args.level_cam:
        level_cam()
    m = snap(args.out.replace(".png", "_f.jpg"))
    perceive.montage([args.out.replace(".png", "_f.jpg")], args.out)
    print(json.dumps({"sense": _c(m)}))
    return _c(m)


def do_turn(args):
    if args.level_cam:
        level_cam()
    frames, steps = [], []
    f0 = f"{args.out}_s0.jpg"
    m = snap(f0)
    frames.append(f0)
    total = 0.0
    aborted = None
    for i in range(1, args.n + 1):
        # gate: an arc pulse lunges ~120mm forward first
        if m["bands"]["center"] < args.min_clear:
            aborted = f"pre-pulse {i}: centre clearance {m['bands']['center']:.2f} < {args.min_clear}"
            break
        car.steer(args.side, args.steer)
        time.sleep(0.25)
        car.move("forward", 55, args.dur)
        car.steer("center", 90)
        time.sleep(0.5)
        fi = f"{args.out}_s{i}.jpg"
        m = snap(fi)
        frames.append(fi)
        dy = yaw(frames[-2], fi)   # + = LEFT/CCW ; right pulse => negative
        total += dy
        steps.append({"pulse": i, "yaw": round(dy, 1), "cum": round(total, 1),
                      "clr": _c(m)})
        print(f"  pulse {i}: yaw {dy:+.1f} (cum {total:+.1f})  "
              f"C={m['bands']['center']:.2f} L={m['bands']['left']:.2f} R={m['bands']['right']:.2f}")
    perceive.montage(frames, args.out)
    summary = {"turn": args.side, "pulses_done": len(steps), "total_yaw": round(total, 1),
               "steps": steps, "aborted": aborted, "montage": args.out}
    print(json.dumps(summary))
    return summary


def do_forward(args):
    if args.level_cam:
        level_cam()
    f0 = f"{args.out}_pre.jpg"
    m = snap(f0)
    c = m["bands"]["center"]
    if c < args.min_clear:
        perceive.montage([f0], args.out)
        print(json.dumps({"forward": 0, "blocked": True,
                          "centre": c, "reason": f"centre {c:.2f} < {args.min_clear}",
                          "montage": args.out}))
        return
    # cap distance by how open it looks (free_frac is not metric -> be conservative)
    mm = args.mm if c >= 0.55 else min(args.mm, 120)
    car.drive_mm("forward", mm)
    time.sleep(0.3)
    f1 = f"{args.out}_post.jpg"
    m2 = snap(f1)
    perceive.montage([f0, f1], args.out)
    print(json.dumps({"forward": mm, "requested": args.mm, "blocked": False,
                      "pre": _c(m), "post": _c(m2), "montage": args.out}))


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("sense"); s.add_argument("out"); s.add_argument("--level-cam", action="store_true")
    t = sub.add_parser("turn")
    t.add_argument("side", choices=["left", "right"]); t.add_argument("n", type=int)
    t.add_argument("out")
    t.add_argument("--dur", type=float, default=0.6); t.add_argument("--steer", type=int, default=60)
    t.add_argument("--min-clear", type=float, default=0.28, dest="min_clear")
    t.add_argument("--level-cam", action="store_true")
    f = sub.add_parser("forward"); f.add_argument("mm", type=int); f.add_argument("out")
    f.add_argument("--min-clear", type=float, default=0.30, dest="min_clear")
    f.add_argument("--level-cam", action="store_true")

    args = p.parse_args()
    if args.cmd == "sense":
        do_sense(args)
    elif args.cmd == "turn":
        do_turn(args)
    elif args.cmd == "forward":
        do_forward(args)


if __name__ == "__main__":
    main()
