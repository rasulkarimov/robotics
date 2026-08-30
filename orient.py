#!/usr/bin/env python3
"""Turn the robot's gaze toward whatever just made a noise, and watch it.

**The microphone cannot tell direction.** The camera's capsule is hardware mono
(ALSA card 3 reports `Channels: 1, Channel map: MONO`), so there is no
time-of-arrival difference to work with and no amount of software will localise
a sound with it. A second microphone would change that: at a 15 cm spacing the
difference is up to ~0.44 ms, which is ~21 samples at 48 kHz - enough for a
coarse left/centre/right. Until then, direction has to come from the eyes.

So this does what an animal with poor ear-localisation does, and what a person
does in a reverberant room: **sweep, and look for what changed.** The bearing
whose view differs most from the remembered room is where something happened.
Then dwell there and keep watching until it goes still.

    orient.py baseline          # remember the quiet room (do this first)
    orient.py look              # sweep, report the most-changed bearing
    orient.py watch             # look, then dwell on it and follow movement

`bearing_from_audio()` is the seam: when a stereo mic arrives, it returns a real
direction and the sweep becomes a confirmation instead of a search.
"""
import argparse
import json
import os
import subprocess
import sys
import time

from PIL import Image, ImageChops, ImageFilter

REPO = os.path.dirname(os.path.abspath(__file__))
BASELINE_DIR = os.path.join(REPO, "nav_state", "room_baseline")
STATE = os.path.join(BASELINE_DIR, "baseline.json")
# Frames from the previous sweep. Orienting is about what changed JUST NOW, not
# about what differs from a memory taken an hour ago: a person who sat down and
# stayed put keeps differing from the baseline forever, so the gaze kept being
# dragged back to them whatever the noise. Comparing with the last sweep makes a
# still person uninteresting and a new movement interesting, which is the whole
# point of an orienting response.
LAST_DIR = os.path.join(REPO, "nav_state", "room_last")

VENV_PY = "/home/astra/tools/venv/bin/python3"
ARM = os.path.join(REPO, "arm.py")

# The lookout shape, horizon pitch: this is about seeing the ROOM, not the floor.
# Only the base turns - servos 3/4 stay put, which is what keeps the arm folded
# and out of the sonar beam.
SHAPE = "3:237,4:843,5:780"
BEARINGS = [170, 290, 410, 470, 530, 650, 770]

# A downscaled greyscale difference: big enough to see a person move, small
# enough that sensor noise and auto-exposure wobble average out.
THUMB = (80, 60)
CHANGE_FLOOR = 6.0     # mean abs difference below this is just noise/exposure


_last_bearing = None


def _frame(bearing, path, settle_ms=None):
    """Move to a bearing and grab a frame.

    The settle time has to scale with the swing. Measured 2026-08-30: a 600-unit
    base sweep taken at a flat 700 ms captured the arm still moving, and the
    blur read as a change of 13.2 - above the 6.0 threshold, i.e. a phantom
    event every time the sweep started from the far side.
    """
    global _last_bearing
    if settle_ms is None:
        swing = abs(bearing - _last_bearing) if _last_bearing is not None else 600
        settle_ms = int(600 + swing * 2.5)      # 600 units -> 2100 ms
    started = time.time()
    subprocess.run(["sudo", VENV_PY, ARM, "step", f"{SHAPE},6:{bearing}",
                    path, str(settle_ms)],
                   cwd=REPO, capture_output=True, text=True, timeout=90)
    _last_bearing = bearing
    # A file that merely EXISTS proves nothing: /tmp keeps the last run's frame,
    # so when the camera was down on 2026-08-30 every capture "succeeded" and the
    # sweep compared stale images with themselves - reporting a confident
    # "nothing has changed" from data minutes old. Freshness is the real test.
    try:
        return os.path.getmtime(path) >= started
    except OSError:
        return False


def _thumb(path):
    im = Image.open(path).convert("L").resize(THUMB)
    return im.filter(ImageFilter.GaussianBlur(1))


def _difference(a, b):
    d = ImageChops.difference(a, b)
    px = list(d.getdata())
    return sum(px) / len(px)


def bearing_from_audio():
    """Direction of the last sound, or None when the hardware cannot say.

    Returns None today: the capsule is mono. This is the ONLY place that has to
    change when a stereo microphone is fitted - everything below already accepts
    a hint and falls back to a full sweep without one.
    """
    return None


def cmd_baseline(args):
    """Remember the quiet room, and how much each bearing wobbles on its own.

    A single threshold does not fit every direction. The bearings facing the
    balcony window re-shoot several points higher than the dark ones, because
    auto-exposure hunts on a blown-out window. A global floor either false-fires
    on the bright side or goes deaf on the dark side, so each bearing gets its
    own, measured.

    Measured over TWO FULL PASSES, not two frames back to back. Arriving at a
    bearing from the far side is what actually happens in service, and it
    carries the settle and the exposure adaptation with it; a back-to-back probe
    reported 0.2 for a bearing that really varies by 11.5, which then flagged as
    an event on every quiet sweep.
    """
    os.makedirs(BASELINE_DIR, exist_ok=True)
    saved = {}
    for b in BEARINGS:
        p = os.path.join(BASELINE_DIR, f"b{b}.jpg")
        if _frame(b, p):
            saved[str(b)] = p
            print(f"  pass 1, bearing {b}: {p}")

    noise = {}
    for b in BEARINGS:                       # second pass, same order, same arrivals
        if str(b) not in saved:
            continue
        probe = f"/tmp/baseline_probe_{b}.jpg"
        if not _frame(b, probe):
            continue
        try:
            noise[str(b)] = round(_difference(_thumb(saved[str(b)]), _thumb(probe)), 2)
        except Exception:
            noise[str(b)] = 0.0
        print(f"  pass 2, bearing {b}: self-noise {noise[str(b)]:.1f}")

    subprocess.run(["sudo", VENV_PY, ARM, "home"], cwd=REPO, capture_output=True)
    with open(STATE, "w") as f:
        json.dump({"taken": time.strftime("%Y-%m-%dT%H:%M:%S"),
                   "shape": SHAPE, "frames": saved, "self_noise": noise}, f,
                  indent=2)
    print(f"baseline of {len(saved)} bearings saved")
    return 0


def sweep_for_change(hint=None):
    """Return [(bearing, change, frame_path)], most changed first."""
    if not os.path.exists(STATE):
        print("no baseline yet - run `orient.py baseline` in a quiet room",
              file=sys.stderr)
        return []
    st = json.load(open(STATE))
    base = st["frames"]
    noise = st.get("self_noise", {})
    os.makedirs(LAST_DIR, exist_ok=True)
    order = BEARINGS
    if hint is not None:
        order = sorted(BEARINGS, key=lambda b: abs(b - hint))
    out = []
    for b in order:
        ref = base.get(str(b))
        if not ref or not os.path.exists(ref):
            continue
        now = f"/tmp/orient_{b}.jpg"
        if not _frame(b, now):
            continue
        prev = os.path.join(LAST_DIR, f"b{b}.jpg")
        against = prev if os.path.exists(prev) else ref
        try:
            change = _difference(_thumb(against), _thumb(now))
            base_change = _difference(_thumb(ref), _thumb(now))
        except Exception:
            continue
        # A bearing only counts as changed once it clears its OWN wobble.
        floor = max(CHANGE_FLOOR, 3.0 * noise.get(str(b), 0.0))
        out.append((b, change, now, base_change))
        src = "since last look" if against is not prev or os.path.exists(prev) else "vs baseline"
        print(f"  bearing {b}: change {change:.1f} {src}  (floor {floor:.1f})"
              + ("  <-- CHANGED" if change >= floor else ""))
        try:
            import shutil
            shutil.copyfile(now, prev)      # this sweep becomes the next one's reference
        except Exception:
            pass
    # Rank by how far each bearing clears ITS OWN floor, not by raw change.
    # The window bearings legitimately wobble ten points while a dark corner
    # wobbles one, so the largest raw number is routinely the least interesting.
    if not out:
        # Say so. Returning an empty list quietly is what made a dead camera
        # look like a quiet room on 2026-08-30.
        print("no bearing produced a fresh frame - is the camera up? "
              "(`car.py status`)", file=sys.stderr)
    out.sort(key=lambda t: -(t[1] / max(bearing_floor(t[0]), 1e-6)))
    return out


# If EVERY bearing looks different, the room did not change - the robot did.
# Measured 2026-08-30: the same room re-shot from the same spot gives 1.4-3.8
# per bearing; after the robot was moved, all seven read 42-74. So a high
# median across bearings is the signature of a stale baseline, and crying
# "something happened!" about it would be a lie seven times over.
MOVED_MEDIAN = 20.0


def bearing_floor(bearing):
    """Change threshold for one bearing: its own measured wobble, times three."""
    try:
        noise = json.load(open(STATE)).get("self_noise", {})
    except Exception:
        return CHANGE_FLOOR
    return max(CHANGE_FLOOR, 3.0 * noise.get(str(bearing), 0.0))


def baseline_is_stale(ranked):
    if len(ranked) < 3:
        return False
    vals = sorted(t[3] for t in ranked)   # "have I moved?" is a BASELINE question
    return vals[len(vals) // 2] >= MOVED_MEDIAN


def cmd_look(args):
    ranked = sweep_for_change(bearing_from_audio())
    if not ranked:
        return 2
    if baseline_is_stale(ranked):
        med = sorted(t[3] for t in ranked)[len(ranked) // 2]
        print(f"every bearing differs (median {med:.1f}) - I have been moved, "
              f"the baseline is stale. Re-run `orient.py baseline`.")
        subprocess.run(["sudo", VENV_PY, ARM, "home"], cwd=REPO, capture_output=True)
        return 3
    b, change, path = ranked[0][:3]
    if change < bearing_floor(b):
        print(f"nothing has changed in the room "
              f"(best {change:.1f} < {bearing_floor(b):.1f})")
        subprocess.run(["sudo", VENV_PY, ARM, "home"], cwd=REPO, capture_output=True)
        return 1
    print(f"looking at bearing {b} (change {change:.1f}) -> {path}")
    _frame(b, "/tmp/orient_target.jpg", settle_ms=900)
    return 0


def is_person(frame, timeout=200):
    """Is there a person in this frame? None when the question could not be answered.

    Costs about a minute, so it is asked ONCE per watch and never per frame.
    Parses the JSON body rather than the exit code: vision.py exits 2 both for
    "not there" and for a reply it could not read.
    """
    try:
        r = subprocess.run(["python3", "vision.py", "find", frame,
                            "a person, or part of a person such as a leg, arm or head"],
                           cwd=REPO, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None
    for line in reversed(r.stdout.strip().splitlines()):
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            obj = json.loads(line)
        except ValueError:
            continue
        if str(obj.get("why", "")).startswith("unparseable reply"):
            return None
        return bool(obj.get("found"))
    return None


def cmd_watch(args):
    """Dwell on the changed bearing and report movement until it goes still.

    Habituation is the point of the still-count: a curtain moving in a draught
    stops being interesting, and a robot that stares at it forever is both silly
    and expensive on a shared battery.
    """
    ranked = sweep_for_change(bearing_from_audio())
    if not ranked:
        return 2
    if baseline_is_stale(ranked):
        print("baseline is stale - I have been moved. Re-run `orient.py baseline`.")
        subprocess.run(["sudo", VENV_PY, ARM, "home"], cwd=REPO, capture_output=True)
        return 3
    b, change = ranked[0][0], ranked[0][1]
    if change < bearing_floor(b):
        print("nothing to watch")
        subprocess.run(["sudo", VENV_PY, ARM, "home"], cwd=REPO, capture_output=True)
        return 1

    print(f"watching bearing {b} (change {change:.1f})")

    # A person is not a curtain. Movement is what earns attention in the first
    # place, but once a PERSON has been found, going still is not a reason to
    # look away - a robot that loses interest in someone the moment they stop
    # moving reads as indifferent, which is the opposite of what this is for.
    # Asked once: the question costs about a minute.
    first = "/tmp/watch_first.jpg"
    _frame(b, first, settle_ms=400)
    person = is_person(first) if not args.no_person_check else None
    deadline = None
    if person:
        deadline = time.time() + args.person_seconds
        print(f"a person is there - holding the gaze for up to "
              f"{args.person_seconds:.0f}s even if they go still")

    prev, still = None, 0
    for i in range(args.frames):
        p = f"/tmp/watch_{i}.jpg"
        if not _frame(b, p, settle_ms=400):
            break
        cur = _thumb(p)
        if prev is not None:
            d = _difference(prev, cur)
            moving = d >= CHANGE_FLOOR
            print(f"  {i}: motion {d:.1f} {'MOVING' if moving else 'still'}")
            still = 0 if moving else still + 1
            if deadline is not None:
                if time.time() >= deadline:
                    print("held the gaze long enough - back to neutral")
                    break
            elif still >= args.patience:
                print(f"gone still for {still} frames - losing interest")
                break
        prev = cur
        time.sleep(args.interval)
    subprocess.run(["sudo", VENV_PY, ARM, "home"], cwd=REPO, capture_output=True)
    who = {True: "person", False: "no person", None: "unknown"}[person]
    print(f"WATCHED bearing {b}, {who}, {i + 1} frames; last view /tmp/watch_{i}.jpg")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("baseline", help="remember the quiet room")
    sub.add_parser("look", help="sweep and point at the most-changed bearing")
    w = sub.add_parser("watch", help="look, then dwell and follow movement")
    w.add_argument("--frames", type=int, default=12)
    w.add_argument("--interval", type=float, default=1.5)
    w.add_argument("--patience", type=int, default=3,
                   help="still frames before losing interest (no person present)")
    w.add_argument("--person-seconds", type=float, default=60.0,
                   help="how long to keep a person in view even when still")
    w.add_argument("--no-person-check", action="store_true",
                   help="skip the ~60 s vision call")
    args = ap.parse_args()
    return {"baseline": cmd_baseline, "look": cmd_look, "watch": cmd_watch}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main() or 0)
