#!/usr/bin/env python3
"""Wake the autonomous operator when the room makes a noise.

The camera (349c:3307) carries a USB microphone on ALSA card 3. Measured
2026-08-30 with the capture gain already at maximum: a silent room sits at
RMS ~11 of 32767, and a clap plus a spoken phrase beside the robot peaked at
27107 with a 0.2 s window RMS of 1373. Two orders of magnitude between floor
and event, which is why a fixed threshold works here and needs no cleverness.

Runs under the SYSTEM python3, like car.py and vision.py. No numpy: a 0.1 s
window is 1600 samples, and `array` handles that comfortably.

    listen.py levels                 # print live RMS, wake nothing (tuning)
    listen.py calibrate              # measure the room's floor for 10 s
    listen.py watch --dry-run        # detect events, log them, wake nothing
    listen.py watch                  # detect events and wake Hermes

Audio is a separate USB interface from video, so this does not disturb
mjpg-streamer on port 8090 - verified while the camera was streaming.
"""
import argparse
import array
import csv
import math
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(REPO, "sound_log.csv")

DEVICE = "plughw:3,0"
RATE = 16000
WINDOW = 0.1                      # seconds per RMS window
SAMPLES = int(RATE * WINDOW)

# Between the measured floor (~11) and a real event (~1373). Far enough from
# both that neither room tone nor a quiet fan can reach it, and a clap or a
# raised voice clears it by a wide margin.
THRESHOLD = 250.0

# One clap must not become five wake-ups: an event is a rising edge, and the
# operator is slow and expensive to run.
COOLDOWN_S = 60.0
MAX_WAKES_PER_HOUR = 6

# Waking the operator costs battery on a pack shared with the Pi, and the
# operator cannot recharge itself yet (ladder step 6). Below this, log the
# event and stay quiet.
BATT_MIN_V = 6.9


def _reader(device=DEVICE):
    """Yield RMS per window from a raw arecord stream."""
    cmd = ["arecord", "-D", device, "-f", "S16_LE", "-r", str(RATE),
           "-c", "1", "-t", "raw", "-q"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    nbytes = SAMPLES * 2
    try:
        while True:
            buf = p.stdout.read(nbytes)
            if not buf or len(buf) < nbytes:
                return
            a = array.array("h")
            a.frombytes(buf)
            yield math.sqrt(sum(x * x for x in a) / len(a)), max(abs(min(a)), abs(max(a)))
    finally:
        p.terminate()
        try:
            p.wait(timeout=2)
        except subprocess.TimeoutExpired:
            p.kill()


def battery_v():
    try:
        r = subprocess.run(["./arm", "battery"], cwd=REPO, capture_output=True,
                           text=True, timeout=40)
    except subprocess.TimeoutExpired:
        return None
    for tok in r.stdout.replace("battery:", " ").split():
        try:
            v = float(tok)
        except ValueError:
            continue
        if 3.0 < v < 12.0:
            return v
    return None


def log_event(rms, peak, action, detail=""):
    new = not os.path.exists(LOG)
    with open(LOG, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["ts", "rms", "peak", "action", "detail"])
        w.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"), f"{rms:.1f}", peak,
                    action, detail])


def wake_hermes(rms, peak):
    """Hand the operator a bounded task. It is told NOT to drive: a noise is a
    reason to look, never a reason to move something it has not checked."""
    prompt = (
        f"Событие: микрофон робота зафиксировал звук (RMS {rms:.0f}, пик {peak}, "
        f"порог {THRESHOLD:.0f}, тишина в комнате даёт около 11).\n\n"
        "Осмотрись и доложи, что происходит. Порядок:\n"
        "1. nav.py lookout --view horizon --snapshot /tmp/sound_look.jpg — "
        "посмотри на комнату (наклон руками не подбирай).\n"
        "2. python3 vision.py describe /tmp/sound_look.jpg — что видно.\n"
        "3. Если в кадре человек — поздоровайся и спроси, нужно ли что-то.\n\n"
        "ШАССИ НЕ ДВИГАТЬ. Звук — повод посмотреть, а не повод ехать. "
        "Если считаешь, что нужно движение — доложи и жди указания.\n"
        "Строку о происшествии запиши в training_log.csv (ступень 0)."
    )
    try:
        subprocess.Popen(["hermes", "-z", prompt], cwd=REPO,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except Exception as e:
        log_event(rms, peak, "wake_failed", str(e)[:120])
        return False


def cmd_levels(args):
    print(f"device {args.device}, window {WINDOW}s, threshold {THRESHOLD}")
    print("rms      peak    bar")
    for rms, peak in _reader(args.device):
        bar = "#" * min(60, int(rms / 40))
        print(f"{rms:7.1f} {peak:6d}  {bar}")


def cmd_calibrate(args):
    vals = []
    t0 = time.time()
    for rms, _ in _reader(args.device):
        vals.append(rms)
        if time.time() - t0 >= args.seconds:
            break
    vals.sort()
    med = vals[len(vals) // 2]
    p95 = vals[int(len(vals) * 0.95)]
    print(f"windows={len(vals)}  median={med:.1f}  p95={p95:.1f}  max={vals[-1]:.1f}")
    print(f"current THRESHOLD={THRESHOLD:.0f} -> "
          f"{'OK, well clear of the floor' if p95 * 4 < THRESHOLD else 'TOO CLOSE to the floor, raise it'}")
    return 0


def cmd_watch(args):
    last_wake = 0.0
    wakes = []
    armed = True          # rising-edge detector: re-arms once the room goes quiet
    print(f"listening on {args.device}, threshold {THRESHOLD}, "
          f"cooldown {COOLDOWN_S}s, dry_run={args.dry_run}", flush=True)
    for rms, peak in _reader(args.device):
        if rms < THRESHOLD:
            armed = True
            continue
        if not armed:
            continue
        armed = False
        now = time.time()

        if now - last_wake < COOLDOWN_S:
            log_event(rms, peak, "suppressed_cooldown")
            continue
        wakes = [t for t in wakes if now - t < 3600]
        if len(wakes) >= MAX_WAKES_PER_HOUR:
            log_event(rms, peak, "suppressed_rate_limit",
                      f"{len(wakes)} in the last hour")
            continue
        if args.dry_run:
            log_event(rms, peak, "detected_dry_run")
            print(f"event rms={rms:.0f} peak={peak} (dry run)", flush=True)
            continue

        v = battery_v()
        if v is None:
            log_event(rms, peak, "suppressed_battery_unreadable")
            continue
        if v < BATT_MIN_V:
            log_event(rms, peak, "suppressed_low_battery", f"{v} V")
            continue

        if wake_hermes(rms, peak):
            last_wake = now
            wakes.append(now)
            log_event(rms, peak, "woke_hermes", f"{v} V")
            print(f"event rms={rms:.0f} peak={peak} -> woke Hermes", flush=True)


def main():
    global THRESHOLD
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default=DEVICE)
    ap.add_argument("--threshold", type=float, default=THRESHOLD,
                    help="RMS above which a window counts as an event; the "
                         "default is calibrated against a measured floor of ~12")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("levels", help="print live RMS; wake nothing")
    c = sub.add_parser("calibrate", help="measure the room's noise floor")
    c.add_argument("--seconds", type=float, default=10.0)
    w = sub.add_parser("watch", help="detect events and wake the operator")
    w.add_argument("--dry-run", action="store_true",
                   help="log events but wake nothing")
    args = ap.parse_args()
    THRESHOLD = args.threshold
    return {"levels": cmd_levels, "calibrate": cmd_calibrate,
            "watch": cmd_watch}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main() or 0)
