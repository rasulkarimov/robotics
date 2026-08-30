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
import collections
import csv
import math
import os
import subprocess
import sys
import threading
import time
import wave

REPO = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(REPO, "sound_log.csv")

DEVICE = "plughw:3,0"
RATE = 16000
WINDOW = 0.1                      # seconds per RMS window
SAMPLES = int(RATE * WINDOW)

# Measured, not guessed. A silent room runs 10-18. A clap right beside the robot
# hits 1373 - but the SAME noise from across the room only reaches 51, because a
# small capsule loses about 27x over that distance. A threshold of 250 was deaf
# to anything that was not happening at arm's length, which is not much use in a
# robot meant to notice the room.
#
# 30 sits at ~1.7x the quiet room's maximum and ~1.7x below a far-side noise.
# That margin is thin by design: the response to a trigger is a cheap local
# sweep, no model call, so an occasional look at nothing costs seconds. Two
# consecutive windows are required, which is what keeps a single click out.
THRESHOLD = 30.0
CONSECUTIVE = 2

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
            yield (math.sqrt(sum(x * x for x in a) / len(a)),
                   max(abs(min(a)), abs(max(a))), buf)
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


def dwell(timeout=400):
    """Stay with whatever the sweep found, instead of glancing and walking away.

    Added 2026-08-30 after the user asked why the robot was not watching them:
    the reflex was only ever calling `look`, so it turned, reported and went back
    to neutral. Looking and then losing interest a second later is not an
    orienting response, it is a twitch.
    """
    try:
        r = subprocess.run(["python3", "orient.py", "watch"], cwd=REPO,
                           capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return "watch timed out"
    for line in reversed(r.stdout.strip().splitlines()):
        if line.startswith("WATCHED"):
            return line
    return (r.stdout.strip().splitlines() or ["watch produced nothing"])[-1]


# Speech is the point of hearing at all. A loud noise says something happened;
# a transcript says what was wanted. whisper.cpp with ggml-base-q5_1 runs at
# roughly 1.8x realtime on this Pi 4 - measured, 9 s wall for a 5 s clip - so a
# few seconds of audio is affordable once per event, and nothing like affordable
# continuously.
# small, not base. Measured 2026-08-30 on one real command, same audio file:
#   base-q5_1  -> "Астра — дымни, сни, брост."      20.3 s
#   small-q5_1 -> "Астра — подними синий брус."     36.0 s
# Base could not reach the phrase even with the word in the vocabulary prompt.
# A command that is transcribed wrong is worse than one that is slow: the
# operator acts on it.
WHISPER_MODEL = "/home/astra/whisper-models/ggml-small-q5_1.bin"
PRE_ROLL_S = 2.0        # audio kept from BEFORE the trigger: a phrase starts
                        # before it gets loud enough to cross the threshold
POST_ROLL_S = 3.0


def transcribe(pre, post, path="/tmp/heard.wav"):
    """Write the audio around an event and return what was said, or ''."""
    try:
        with wave.open(path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(RATE)
            w.writeframes(b"".join(list(pre) + post))
    except Exception as e:
        return f"(could not write audio: {e})"
    try:
        r = subprocess.run(["./stt.sh", path, "ru"], cwd=REPO,
                           capture_output=True, text=True, timeout=180,
                           env={**os.environ, "WHISPER_MODEL": WHISPER_MODEL})
    except subprocess.TimeoutExpired:
        return "(transcription timed out)"
    text = " ".join(r.stdout.split())
    # whisper marks non-speech like this; it is an answer, not a failure.
    if text in ("[музыка]", "[Music]", "[BLANK_AUDIO]", ""):
        return ""
    return text


def orient(timeout=180):
    """Reflex before thought: sweep and see whether anything actually changed.

    Ears, then eyes, then brain - in that order, the way an animal does it. The
    sweep is local, takes about ten seconds and costs no model call, so a noise
    with nothing behind it (a door two rooms away, a car outside) is answered
    with a look and nothing more. Only a noise that came with a visible change
    is worth the operator's time and the battery a session costs.

    Returns (changed, detail).
    """
    try:
        r = subprocess.run(["python3", "orient.py", "look"], cwd=REPO,
                           capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None, "orient timed out"
    # The "camera is down" message goes to stderr, so reading stdout alone
    # recorded a useful diagnosis as the useless string "no output".
    tail = [l for l in (r.stdout + "\n" + r.stderr).strip().splitlines() if l.strip()]
    detail = tail[-1] if tail else "no output"
    if r.returncode == 0:
        return True, detail
    if r.returncode == 1:
        return False, detail
    if r.returncode == 3:
        return "stale", detail   # the robot was moved; the baseline means nothing
    return None, detail          # no baseline, or the sweep failed


def log_training_row(rms, peak, seen, said, watched):
    """Write the training-log row ourselves, from what was actually measured.

    Asking the operator to write it did not work. Four runs in a row invented
    distances - "20-40 см", "~1 м", "~60 см", "~20cm" - and the last of those
    came AFTER an explicit ban on writing distances at all; the same run also
    stamped the row 12:10 while the clock said 11:59. None of it was malice: it
    was writing a report from an impression, in a column that asks for
    measurements.
    #
    # So the opportunity is removed rather than the instruction repeated. This
    # process has the real clock and the real numbers - RMS, peak, threshold,
    # the bearing the sweep chose, whether vision found a person, and the
    # transcript - so it writes the row, and the operator is told to report to
    # the human instead.
    """
    person = "person" if ", person," in watched else (
        "no person" if ", no person," in watched else "person unknown")
    bearing = ""
    for tok in seen.replace("(", " ").replace(")", " ").split():
        if tok.isdigit() and len(tok) == 3:
            bearing = f", bearing {tok}"
            break
    note = person + (f'; heard: "{said}"' if said else "; no speech")
    row = [time.strftime("%Y-%m-%dT%H:%M:%S"), "0",
           "sound wake: orient and watch",
           f"RMS {rms:.0f}, peak {peak}, threshold {THRESHOLD:.0f}{bearing}",
           "pass", note]
    path = os.path.join(REPO, "training_log.csv")
    try:
        with open(path, "a", newline="") as f:
            csv.writer(f).writerow(row)
    except Exception as e:
        log_event(rms, peak, "training_log_write_failed", str(e)[:120])


def wake_hermes(rms, peak, seen="", said=""):
    """Hand the operator a bounded task. It is told NOT to drive: a noise is a
    reason to look, never a reason to move something it has not checked."""
    prompt = (
        f"Событие: микрофон робота зафиксировал звук (RMS {rms:.0f}, пик {peak}, "
        f"порог {THRESHOLD:.0f}, тишина в комнате даёт около 11).\n"
        f"Рефлекс уже отработал: {seen}\n"
        + (f'РАСПОЗНАННАЯ РЕЧЬ: "{said}"\nЕсли это просьба — выполни её или '
           f"скажи, что мешает.\n" if said else "")
        +
        "Кадр с этого азимута лежит в /tmp/orient_target.jpg.\n\n"
        "Посмотри и доложи, что происходит. Порядок:\n"
        "1. python3 vision.py describe /tmp/orient_target.jpg — что видно.\n"
        "Наблюдение УЖЕ проведено рефлексом — orient.py watch повторно не "
        "запускай, рука уже вернулась в home.\n"
        "3. Если в кадре человек — поздоровайся и спроси, нужно ли что-то.\n\n"
        "ШАССИ НЕ ДВИГАТЬ. Звук — повод посмотреть, а не повод ехать. "
        "Если считаешь, что нужно движение — доложи и жди указания.\n"
        "В training_log.csv НЕ ПИШИ — строку уже записал демон, у него есть "
        "часы и измерения. Доложи человеку словами.\n"
        "(справочно, формат файла: 6 колонок в этом "
        "порядке, свой заголовок НЕ добавляй:\n"
        "ts,step,what_was_tried,measured,verdict,note\n"
        "step — НОМЕР ступени (пробуждение по звуку = 0), не слово. "
        "В measured — только измеренное. РАССТОЯНИЕ НЕ ПИШИ ВООБЩЕ: измерить "
        "его на этом азимуте нечем (сонар смотрит вперёд, у камеры глубины нет). "
        "Описывай ЧТО видишь и ГДЕ в кадре. Дистанция допустима только рядом с "
        "показанием car.py ultrasonic.\n"
        "Пример: 2026-08-30T10:51:09,0,\"orient toward a sound\","
        "\"RMS 421, peak 1446\",pass,\"человек справа в 1.5 м\"\n"
        "Файл — общая доказательная база; вторая раскладка колонок ломает разбор "
        "и ведущему приходится чинить журнал вручную."
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
    for rms, peak, _ in _reader(args.device):
        bar = "#" * min(60, int(rms / 40))
        print(f"{rms:7.1f} {peak:6d}  {bar}")


def cmd_calibrate(args):
    vals = []
    t0 = time.time()
    for rms, _, _ in _reader(args.device):
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
    run = 0               # consecutive windows over threshold
    pre = collections.deque(maxlen=int(PRE_ROLL_S / WINDOW))
    print(f"listening on {args.device}, threshold {THRESHOLD}, "
          f"cooldown {COOLDOWN_S}s, dry_run={args.dry_run}", flush=True)
    reader = _reader(args.device)
    for rms, peak, buf in reader:
        pre.append(buf)
        if rms < THRESHOLD:
            run = 0
            armed = True
            continue
        run += 1
        if run < CONSECUTIVE:
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

        # Catch the tail of whatever is being said before turning to look: the
        # arm sweep is noisy and takes 40 s, by which time the sentence is gone.
        post = []
        for _ in range(int(POST_ROLL_S / WINDOW)):
            try:
                post.append(next(reader)[2])
            except StopIteration:
                break
        # Transcribing and sweeping do not compete: one is CPU, the other is
        # the arm and the camera. Run them together, or the robot spends 36 s
        # listening to a recording before it starts turning its head.
        heard = {}
        t = threading.Thread(target=lambda: heard.update(text=transcribe(pre, post)),
                             daemon=True)
        t.start()

        # Log the detection BEFORE responding. The sweep takes ~40 s, and until
        # 2026-08-30 nothing was written until it finished - so anyone watching
        # the log during a response saw an empty file and concluded the robot
        # had not heard them.
        log_event(rms, peak, "detected", f"{v} V, responding")
        changed, detail = orient()

        t.join(timeout=240)
        said = heard.get("text", "")
        if said:
            print(f'  heard: "{said}"', flush=True)
        if changed is False and not said:
            # Looked, saw nothing, heard no words. That IS the whole response.
            log_event(rms, peak, "oriented_nothing_seen", detail)
            print(f"event rms={rms:.0f} -> looked, nothing changed", flush=True)
            last_wake = now          # the look itself earns the cooldown
            continue
        if changed == "stale":
            log_event(rms, peak, "baseline_stale", detail)
            print("event -> baseline stale (robot was moved); not waking", flush=True)
            last_wake = now
            continue
        if changed is None:
            log_event(rms, peak, "orient_failed", detail)

        # Reflex first, and see it through: turn, then stay with it. Only then
        # is there anything worth telling the operator.
        watched = dwell()
        print(f"  {watched}", flush=True)

        log_training_row(rms, peak, detail, said, watched)
        if wake_hermes(rms, peak, f"{detail}; {watched}", said):
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
