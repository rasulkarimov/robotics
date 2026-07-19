#!/usr/bin/env python3
"""CLI for controlling the Hiwonder xArm over USB HID, via the `xarm` PyPI package.

Setup notes (so they aren't rediscovered):
- The control board enumerates as USB HID (VID 0483 / PID 5750), NOT a serial port -
  no /dev/ttyUSB*. lsusb misidentifies it as "STMicroelectronics LED badge" because
  Hiwonder reuses a generic STM32 HID VID:PID pair; dmesg confirms the real identity
  (Manufacturer: Hiwonder, Product: xArm).
- Protocol: 0x55 0x55 <len> <cmd> <params...> wrapped in 64-byte HID reports (same
  framing as Hiwonder's serial bus-servo protocol, documented at docs.hiwonder.com).
  The `xarm` package + `hidapi` package implement this - use them, don't hand-roll it.
- Lives in a venv (/home/astra/tools/venv) because pip install is blocked system-wide
  (PEP 668). Always invoke via /home/astra/tools/venv/bin/python3. If the venv is ever
  missing (e.g. after an SD card / environment reset), recreate it with:
    python3 -m venv /home/astra/tools/venv
    /home/astra/tools/venv/bin/pip install -r requirements.txt
- A udev rule (/etc/udev/rules.d/99-hiwonder-xarm.rules) sets /dev/hidraw* for this
  VID:PID to mode 0666, but hidapi's open-by-VID/PID still failed as non-root for
  reasons not fully root-caused (plain file open() worked fine, so it's something
  hidapi-specific, not a bare permission bit issue). Didn't chase it further - just
  run this script with sudo, matching how the car's own server self-elevates for
  mjpg-streamer. If you do want to debug the non-root path, that's where to start.
- Servo position units: 0-1000 maps linearly to -125.0..+125.0 degrees, 500/0.0 is
  center. Verify actual mechanical range per joint before trusting the full span -
  the xarm library does NOT know your arm's real joint limits, it will happily
  command a position that's mechanically unreachable and stall/strain a servo.
- ALWAYS read current positions before the first move on a new session (`status`),
  and prefer small relative moves with generous duration (>=800ms) over big jumps.

Joint map (CORRECTED 2026-07-11 by direct live user confirmation while hand-teaching;
supersedes an earlier version that had 1 and 2 SWAPPED. Do not re-swap them):
    1 = gripper/claw   - open/close. ~156 = fully OPEN, ~686 = fully CLOSED (empty,
        hard mechanical stop - the "stop at ~686-689" I earlier misattributed to a
        wrist joint is really the jaws bottoming out). This object closed the jaws
        at ~526. KEY GOTCHA: servo 1 IS back-drivable, so hand-closing the claw
        during `teach`/`capture` DOES record correctly on servo 1 - but commanding
        it PAST the object (toward fully-closed) stalls and reads unchanged; that's
        normal (jaws blocked by the object), not a dead servo.
    2 = wrist rotate   - rotates the gripper's orientation in place. Confirmed live:
        commanding servo 2 = "это ротация" per the user. Subtle from most cameras.
    3 = shoulder       - large effect, swings most of the arm's mass through a big
        arc. Test before assuming exact sign/direction each session.
    4 = elbow          - also a large effect, similar magnitude to shoulder(3).
    5 = wrist pitch/tilt - clear, distinct effect: tilts the forearm+wrist+camera
        down toward the floor. This is the one that got the wrist-mounted camera
        to finally see the floor after `4`/`5` blind sweeps failed to find it.
    6 = base rotation  - confirmed both by the user watching live AND by comparing
        external-camera shots against the floor tile grid lines (base plate
        visibly rotates). The wrist-mounted camera alone made this look like it had
        "no effect" - it does, just not always visible from that camera's own POV.
Do not trust servo-ID-to-joint mappings from Hiwonder's other product docs (ArmPi
FPV, xArm AI) - checked both, neither one's numbering matches this xArm 1S board.
"""
import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request

VENV_PY = "/home/astra/tools/venv/bin/python3"
DEFAULT_SERVO_IDS = [1, 2, 3, 4, 5, 6]
MIN_DURATION_MS = 500  # floor, to avoid jerky/abrupt moves that strain servos/gears
RECORDINGS_DIR = "/home/astra/robotics/arm_recordings"  # in-repo: survives reboots AND is
                                                        # version-controlled (was
                                                        # /home/astra/tools/arm_recordings,
                                                        # outside git - a taught trajectory
                                                        # would die with the SD card)

# Battery guard. The arm may run off 2x 18650 in series (2S): 8.4 V full, 7.4 V nominal,
# and below ~3.0 V/cell under load lithium starts to degrade - under 2.5 V/cell it is
# ruined for good. The board reports its own supply voltage, so there is no excuse for
# flattening a pack by accident. Sag matters: measured 0.53 V drop while moving on a
# half-charged pack, so a reading that looks safe at rest can be deep in the danger zone
# mid-motion. Thresholds are deliberately set on the pack voltage AT REST.
BATT_WARN = 6.8      # ~3.4 V/cell - charge it soon
BATT_STOP = 6.4      # ~3.2 V/cell - refuse to move; sag would push cells under 3.0 V
BATT_ADAPTER = 9.0   # above this it is clearly a mains adapter, not a 2S pack
# With the main power lead unplugged the board still runs, powered over USB, and reports
# the USB rail (~4.5-5.0 V). That is NOT a dying battery - the servos simply have no
# supply. Reporting it as "cells are being destroyed" is alarming and wrong, so treat
# this band as its own case.
USB_ONLY_HI = 5.3

JOINT_NAMES = {
    1: "gripper",       # ~156 open, ~526 closed-on-object, ~686 fully closed (empty)
    2: "wrist_rotate",
    3: "shoulder",
    4: "elbow",
    5: "wrist_pitch",
    6: "base",
}
GRIPPER_OPEN = 156      # servo 1 fully open, measured during teaching
GRIPPER_CLOSED = 640    # servo 1 near fully-closed; will stall earlier if it hits an object

# User-designated default pose (not servo-center 500!). Set via live instruction.
#
# base(6)=470 is the arm's FORWARD reference, fixed by the user on 2026-07-12 with the arm
# mounted on the car: at 470 the claw points straight ahead of the vehicle. Servo centre
# (500) is NOT forward - it is merely the middle of the servo's travel, so "0 degrees" in
# servo terms means nothing physical. Direction convention, also established live:
#     servo 6 UP   -> claw swings LEFT
#     servo 6 DOWN -> claw swings RIGHT
# (~4 units per degree; at a 108 mm reach, 32 units ~= 15 mm sideways.)
HOME_POSE = {1: 506, 2: 499, 3: 237, 4: 843, 5: 682, 6: 470}
BASE_FORWARD = 470   # claw points straight ahead of the car
CAMERA_URL = "http://127.0.0.1:8090/?action=snapshot"


def _in_venv():
    return sys.executable == VENV_PY


def _reexec_with_sudo_venv():
    """We need both the venv's hid/xarm packages AND root for /dev/hidraw access.
    Simplest reliable path: re-exec this same script under `sudo <venv-python>`."""
    os_argv = sys.argv[:]
    subprocess.check_call(["sudo", VENV_PY, __file__] + os_argv[1:])
    sys.exit(0)


def connect(debug=False):
    import xarm
    return xarm.Controller("USB", debug=debug)


def battery(arm):
    return arm.getBatteryVoltage()


def positions(arm, ids=None):
    ids = ids or DEFAULT_SERVO_IDS
    out = {}
    for i in ids:
        try:
            pos = arm.getPosition(i)
            out[i] = pos
        except Exception as e:
            out[i] = f"error: {e}"
    return out


def move_one(arm, servo_id, position, duration_ms=1000, wait=True):
    duration_ms = max(MIN_DURATION_MS, int(duration_ms))
    if not (0 <= position <= 1000):
        raise ValueError("position must be 0-1000 (500 = center)")
    arm.setPosition(servo_id, position, duration=duration_ms, wait=wait)


def release(arm, ids=None):
    """Cut torque so the arm can be moved by hand / rests safely."""
    arm.servoOff(ids)


def snapshot(path, timeout=5):
    with urllib.request.urlopen(CAMERA_URL, timeout=timeout) as r:
        data = r.read()
    with open(path, "wb") as f:
        f.write(data)
    return len(data)


def step(arm, moves, path, duration_ms=1000, settle=0.15):
    """Move one or more servos (simultaneously) then immediately grab a camera
    snapshot - all in this one already-sudo'd process. `moves` is a list of
    (servo_id, position) pairs. This is the fast path: one process, one HID
    session, one settle pause, one HTTP request - instead of separate
    move/sleep/snapshot CLI calls."""
    duration_ms = max(MIN_DURATION_MS, int(duration_ms))
    for servo_id, position in moves:
        if not (0 <= position <= 1000):
            raise ValueError(f"servo {servo_id}: position must be 0-1000")
    if len(moves) == 1:
        arm.setPosition(moves[0][0], moves[0][1], duration=duration_ms, wait=False)
    else:
        arm.setPosition(list(moves), duration=duration_ms, wait=False)
    time.sleep(duration_ms / 1000.0)
    time.sleep(settle)
    size = snapshot(path)
    print(f"step done: {path} ({size} bytes)")
    return size


def battery_guard(arm, force=False):
    """Refuse to drive the servos on a pack that is too flat to take the load.

    Returns True if it is safe to move. Prints a warning as the pack gets low. Pass
    --force to override (e.g. to park the arm safely before charging)."""
    try:
        v = battery(arm)
    except Exception:
        return True                      # can't read it - don't block on that
    if v is None or v > BATT_ADAPTER:
        return True                      # mains adapter, no lithium to protect
    if v <= USB_ONLY_HI:
        print(f"НЕТ СИЛОВОГО ПИТАНИЯ: плата видит {v:.2f} В — это USB, сервоприводы "
              f"обесточены. Подключите питание (адаптер 7.5 В или заряженный 2S).",
              file=sys.stderr)
        return False
    if v < BATT_STOP and not force:
        print(f"АККУМУЛЯТОР РАЗРЯЖЕН: {v:.2f} В (~{v/2:.2f} В/ячейка). Двигаться не буду - "
              f"под нагрузкой просадка уведёт ячейки ниже 3.0 В и убьёт их. "
              f"Зарядите (полный 2S = 8.4 В). Обойти: --force", file=sys.stderr)
        return False
    if v < BATT_WARN:
        print(f"(батарея низкая: {v:.2f} В, ~{v/2:.2f} В/ячейка - скоро заряжать)",
              file=sys.stderr)
    return True


def home(arm, duration_ms=1500, keep_grip=False):
    """Move to the user-designated default pose (HOME_POSE), not servo-center 500.

    keep_grip leaves the gripper alone. HOME_POSE includes servo 1 at 506, i.e. OPEN - so
    a plain `home` while carrying something drops it on the floor. Anything that goes home
    holding an object wants keep_grip=True."""
    pose = {j: v for j, v in HOME_POSE.items() if not (keep_grip and j == 1)}
    arm.setPosition(list(pose.items()), duration=duration_ms, wait=True)


# ---- teach-by-demonstration (kinesthetic teaching) --------------------------
# Core idea: user hand-guides the limp arm; we record joint positions; later we
# replay them with torque on. Solves the depth/coordination problem that pure
# camera-guided grasping kept failing at (a single 2D camera gives no reliable
# forward-distance sense, so aligning the open claw to a floor object was guessing).

def _recording_path(name):
    return os.path.join(RECORDINGS_DIR, f"{name}.json")


def _load_recording(name):
    with open(_recording_path(name)) as f:
        return json.load(f)


def _save_recording(name, data):
    os.makedirs(RECORDINGS_DIR, exist_ok=True)
    with open(_recording_path(name), "w") as f:
        json.dump(data, f, indent=2)


def teach(arm, name, duration=15.0, interval=0.3):
    """Torque OFF, then sample all 6 joint positions every `interval` sec for
    `duration` sec while the user hand-moves the arm through the motion. Saves a
    dense trajectory. At the end, re-engages torque holding the LAST sampled pose
    so the arm freezes where it was left instead of flopping."""
    release(arm)  # torque off - arm goes limp, user can move it by hand
    print(f"RECORDING '{name}': torque OFF. Move the arm by hand now. "
          f"Sampling every {interval}s for {duration}s...")
    waypoints = []
    t0 = time.time()
    while time.time() - t0 < duration:
        pos = {sid: arm.getPosition(sid) for sid in DEFAULT_SERVO_IDS}
        waypoints.append({"t": round(time.time() - t0, 3), "pos": pos})
        time.sleep(interval)
    data = {"name": name, "mode": "trajectory", "interval": interval,
            "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"), "waypoints": waypoints}
    _save_recording(name, data)
    # freeze at last pose so it doesn't drop when the user lets go
    last = waypoints[-1]["pos"]
    arm.setPosition([(sid, p) for sid, p in last.items()], duration=400, wait=True)
    print(f"saved '{name}': {len(waypoints)} waypoints. Torque re-engaged at last pose.")
    return data


def capture(arm, name):
    """Append the CURRENT live joint positions as one keyframe to recording `name`
    (creates it if new). For the cleaner keyframe workflow: user hand-positions the
    arm into a pose (e.g. 'open claw just above object'), we snapshot it, repeat.
    Less jittery than continuous `teach`. Assumes the arm is being held in place
    (or torque left on) at capture time."""
    try:
        data = _load_recording(name)
        if data.get("mode") != "keyframes":
            raise ValueError(f"'{name}' exists as mode '{data.get('mode')}', not keyframes")
    except FileNotFoundError:
        data = {"name": name, "mode": "keyframes",
                "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"), "waypoints": []}
    pos = {sid: arm.getPosition(sid) for sid in DEFAULT_SERVO_IDS}
    data["waypoints"].append({"pos": pos})
    _save_recording(name, data)
    print(f"captured keyframe #{len(data['waypoints'])} for '{name}': {pos}")
    return data


def replay(arm, name, step_ms=None, start_slow_ms=1500):
    """Replay a saved recording with torque on. First waypoint is approached slowly
    (start_slow_ms) so we ease into the start pose from wherever the arm currently
    is; remaining waypoints play at the recording's own cadence (trajectory) or a
    fixed step_ms (keyframes, default 900ms)."""
    data = _load_recording(name)
    wps = data["waypoints"]
    if not wps:
        raise ValueError(f"'{name}' has no waypoints")
    mode = data.get("mode", "trajectory")
    default_step = int((data.get("interval", 0.3)) * 1000) if mode == "trajectory" else 900
    step_ms = step_ms or default_step

    def go(pos, dur):
        arm.setPosition([(int(s), int(p)) for s, p in pos.items()],
                        duration=max(MIN_DURATION_MS, dur), wait=True)

    print(f"replay '{name}' ({mode}, {len(wps)} waypoints)")
    go(wps[0]["pos"], start_slow_ms)  # ease into start pose
    for wp in wps[1:]:
        go(wp["pos"], step_ms)
    print(f"replay '{name}' done")


def list_recordings():
    if not os.path.isdir(RECORDINGS_DIR):
        print("(no recordings yet)")
        return
    for fn in sorted(os.listdir(RECORDINGS_DIR)):
        if fn.endswith(".json"):
            try:
                d = _load_recording(fn[:-5])
                print(f"{fn[:-5]}: mode={d.get('mode')} waypoints={len(d.get('waypoints', []))} "
                      f"recorded_at={d.get('recorded_at')}")
            except Exception as e:
                print(f"{fn[:-5]}: (unreadable: {e})")


def main():
    if not _in_venv():
        _reexec_with_sudo_venv()

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="battery voltage + all servo positions (read-only, safe)")
    sub.add_parser("battery")
    hm = sub.add_parser("home", help="move to the user-designated default pose (HOME_POSE)")
    hm.add_argument("--keep-grip", action="store_true",
                    help="don't touch the gripper (HOME_POSE opens it, dropping anything held)")

    getp = sub.add_parser("get")
    getp.add_argument("servo_id", type=int)

    mv = sub.add_parser("move")
    mv.add_argument("servo_id", type=int)
    mv.add_argument("position", type=int, help="0-1000, 500=center")
    mv.add_argument("duration_ms", type=int, nargs="?", default=1000)

    rel = sub.add_parser("release", help="cut torque (all servos, or specific ids)")
    rel.add_argument("servo_ids", type=int, nargs="*", default=None)

    st = sub.add_parser("step", help="move one or more servos + grab camera snapshot in one call")
    st.add_argument("moves", help="either 'SERVO:POS' or 'SERVO:POS,SERVO:POS,...' for simultaneous moves")
    st.add_argument("path", help="where to save the post-move camera snapshot")
    st.add_argument("duration_ms", type=int, nargs="?", default=1000)
    st.add_argument("--settle", type=float, default=0.15)

    tc = sub.add_parser("teach", help="torque off + record hand-guided motion as a dense trajectory")
    tc.add_argument("name")
    tc.add_argument("duration", type=float, nargs="?", default=15.0, help="seconds to record")
    tc.add_argument("interval", type=float, nargs="?", default=0.3, help="sample period sec")

    cap = sub.add_parser("capture", help="append current pose as one keyframe to a named recording")
    cap.add_argument("name")

    rp = sub.add_parser("replay", help="replay a saved recording (torque on)")
    rp.add_argument("name")
    rp.add_argument("step_ms", type=int, nargs="?", default=None)

    sub.add_parser("recordings", help="list saved recordings")

    p.add_argument("--force", action="store_true",
                   help="move even on a flat battery (only to park the arm)")
    args = p.parse_args()

    arm = connect()

    # Anything that drives the servos goes through the battery guard. Read-only commands
    # (status/battery/get/recordings) always work - you need them most when it's flat.
    if args.cmd in ("move", "step", "home", "replay") and not battery_guard(arm, args.force):
        sys.exit(2)

    if args.cmd == "status":
        v = battery(arm)
        print(f"battery: {v} V")
        for sid, pos in positions(arm).items():
            name = JOINT_NAMES.get(sid, "?")
            if isinstance(pos, int):
                deg = (pos / 1000.0) * 250.0 - 125.0
                print(f"servo {sid} ({name}): position={pos} (~{deg:.1f} deg)")
            else:
                print(f"servo {sid} ({name}): {pos}")
    elif args.cmd == "battery":
        print(f"{battery(arm)} V")
    elif args.cmd == "home":
        home(arm, keep_grip=args.keep_grip)
        print(f"home: {HOME_POSE}" + (" (клешня не тронута)" if args.keep_grip else ""))
    elif args.cmd == "get":
        print(arm.getPosition(args.servo_id))
    elif args.cmd == "move":
        move_one(arm, args.servo_id, args.position, args.duration_ms)
        print(f"servo {args.servo_id} -> {args.position} over {args.duration_ms}ms")
    elif args.cmd == "release":
        ids = args.servo_ids if args.servo_ids else None
        release(arm, ids)
        print(f"released: {ids or 'all'}")
    elif args.cmd == "step":
        moves = []
        for pair in args.moves.split(","):
            sid, pos = pair.split(":")
            moves.append((int(sid), int(pos)))
        step(arm, moves, args.path, args.duration_ms, args.settle)
    elif args.cmd == "teach":
        teach(arm, args.name, args.duration, args.interval)
    elif args.cmd == "capture":
        capture(arm, args.name)
    elif args.cmd == "replay":
        replay(arm, args.name, args.step_ms)
    elif args.cmd == "recordings":
        list_recordings()


if __name__ == "__main__":
    main()
