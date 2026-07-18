#!/usr/bin/env python3
"""CLI for controlling the Freenove three-wheeled car (TCP command protocol + mjpg-streamer HTTP).

Notes learned the hard way this session, baked in here so they aren't rediscovered:
- The TCP server (port 12345) handles ONE connection at a time in a single blocking loop.
  A single connect -> send -> close per call keeps things reliable.
- ">Camera Center" and ">Camera Stop" are no-ops in mTCPServer.py (literally `pass`).
  To actually center the pan/tilt mount, set pan/tilt to angle 90 explicitly.
- ">Camera Left"/">Camera Right" (and Up/Down) both just set the servo to the
  absolute angle given in the command (0-180) - direction in the name doesn't matter,
  the value is not relative.
- ">Turn Left"/">Turn Right" take an offset (10-60) from center (90), matching the
  GUI's slider_Direction range. ">Turn Center<angle>" sets steering to an absolute angle.
- Tilt (SERVO3, "Camera Up"/"Camera Down") behaves oddly on this specific car: 0 and
  180 both give the same clear forward view as the untouched power-on state, but any
  mid-range value (tested 30/60/90/120/150) produced an identical close-up blur.
  Likely no real tilt axis on this 3-wheel model, or a mechanical bind mid-travel.
  Do NOT drive tilt to arbitrary values - camera_center() leaves tilt alone and only
  recenters pan. If you need tilt for something, test 0 and 180 only.
- IMPORTANT (confirmed by user 2026-07-19): despite the "Camera" naming inherited from
  the Freenove kit's protocol, SERVO2/SERVO3 (camera_pan/camera_tilt/camera_center,
  ">Camera Left/Right/Up/Down") now physically drive the ULTRASONIC SENSOR MOUNT, not
  the camera. The actual camera is mounted on the arm (fixed relative to it - see
  perceive.py's "wrist-camera" framing, gripper fingers always in the bottom corners of
  every snapshot). This is why look_around()/pan sweeps produce near-identical frames
  regardless of angle: panning moves the ultrasonic turret, which is out of frame, while
  the camera itself doesn't move. To actually look around the room, move the ARM
  (arm.py) and/or drive the car chassis, not car.py pan/tilt.
- Querying ultrasonic (">Ultrasonic") has, in this session, at times blocked the
  entire server indefinitely if the sensor wiring is bad, wedging ALL car control
  until the server process is killed and restarted. Use --timeout and treat a
  timeout as "may have wedged the server", not just "no reading".
- This Pi has rebooted spontaneously many times this session (every ~5-45 min,
  wiping /tmp each time - that's why this file lives in the home dir, not scratch).
  Always run `status` first before trusting anything is still running. If it's
  down, restart with:
    cd ~/Freenove_Three-wheeled_Smart_Car_Kit_for_Raspberry_Pi/Server
    DISPLAY=:10.0 XAUTHORITY=/home/astra/.Xauthority nohup sh Startup.sh > /tmp/startup.log 2>&1 &
    disown
  (Main.py always creates a QApplication, so it needs a real DISPLAY/XAUTHORITY -
  check `who`/`ps aux | grep Xorg` for the current session's display number if
  :10.0 no longer matches.)
"""
import argparse
import glob
import os
import socket
import subprocess
import sys
import time
import urllib.request

HOST = "127.0.0.1"
CMD_PORT = 12345
CAMERA_PORT = 8090
DEFAULT_TIMEOUT = 5


def port_busy():
    """Is somebody ELSE holding the command port?

    The server accepts exactly ONE connection at a time. While the GUI client is
    connected, our commands are simply never delivered - the car sits there, and it looks
    for all the world like a flat battery or a jammed wheel. That cost real time twice in
    one session, hunting hardware faults that did not exist. Cheap to check, so check."""
    out = subprocess.run(["ss", "-tn"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if f":{CMD_PORT}" in line and "ESTAB" in line:
            return True
    return False


def send(cmd, timeout=DEFAULT_TIMEOUT, wait_response=False):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((HOST, CMD_PORT))
        s.sendall(cmd.encode())
        if wait_response:
            return s.recv(1024).decode(errors="replace")
        return None
    finally:
        s.close()


# Distance calibration, measured 2026-07-12: speed 55 for 0.5 s travelled exactly 100 mm.
# This is ONE data point on ONE floor - the car has no encoders, so this is dead reckoning
# and nothing more. It will drift with battery charge, floor surface, load and tyre wear,
# and a short enough pulse may not overcome static friction at all. Treat `drive_mm` as a
# rough nudge, never as a measurement, and re-check it if precision matters.
CAL_SPEED = 55
CAL_SECONDS = 0.5
CAL_MM = 100.0


def move(direction, speed, duration):
    speed = max(0, min(100, int(speed)))
    word = "Forward" if direction == "forward" else "Backward"
    send(f">Move {word}{speed}")
    time.sleep(float(duration))
    send(">Move Stop")


def drive_mm(direction, mm):
    """Nudge roughly `mm` millimetres. Open-loop - see the calibration note above."""
    secs = CAL_SECONDS * (float(mm) / CAL_MM)
    if secs < 0.25:
        print(f"(пульс {secs:.2f}с очень короткий — машина может не тронуться с места)",
              file=sys.stderr)
    move(direction, CAL_SPEED, secs)
    print(f"{direction} ~{mm:.0f} мм (скорость {CAL_SPEED}, {secs:.2f} с) — без одометрии, "
          f"фактическое расстояние не проверено")


def stop():
    send(">Move Stop")


def steer(direction, angle):
    if direction == "center":
        send(">Turn Center90")
        return
    angle = max(10, min(60, int(angle)))
    word = "Left" if direction == "left" else "Right"
    send(f">Turn {word}{angle}")


def camera_pan(angle):
    """Despite the name, this moves the ULTRASONIC MOUNT, not the camera - see module docstring."""
    angle = max(0, min(180, int(angle)))
    send(f">Camera Left{angle}")


def camera_tilt(angle):
    """Despite the name, this moves the ULTRASONIC MOUNT, not the camera - see module docstring."""
    angle = max(0, min(180, int(angle)))
    send(f">Camera Up{angle}")


def camera_center():
    camera_pan(90)  # tilt deliberately left untouched - see module docstring


def step(direction, speed, duration, path, settle=0.15, steer_dir=None, steer_angle=40, timeout=DEFAULT_TIMEOUT):
    """One combined move+stop+snapshot in a single connection/process - the fast path
    for iterative "nudge, check, nudge, check" driving instead of 3 separate CLI calls."""
    speed = max(0, min(100, int(speed)))
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect((HOST, CMD_PORT))
    try:
        if steer_dir == "center":
            s.sendall(b">Turn Center90")
        elif steer_dir in ("left", "right"):
            angle = max(10, min(60, int(steer_angle)))
            word = "Left" if steer_dir == "left" else "Right"
            s.sendall(f">Turn {word}{angle}".encode())
        word = "Forward" if direction == "forward" else "Backward"
        s.sendall(f">Move {word}{speed}".encode())
        time.sleep(float(duration))
        s.sendall(b">Move Stop")
    finally:
        s.close()
    time.sleep(settle)
    size = snapshot(path)
    print(f"step done: {path} ({size} bytes)")
    return size


def rgb(channel, on):
    cmd = {"r": ">RGB Red", "g": ">RGB Green", "b": ">RGB Blue"}[channel]
    send(cmd)  # server toggles state, no on/off param actually read


def buzzer(on):
    send(">Buzzer Alarm1" if on else ">Buzzer Alarm0")


def ultrasonic(timeout=6):
    try:
        resp = send(">Ultrasonic", timeout=timeout, wait_response=True)
        return resp
    except socket.timeout:
        print("TIMEOUT: no response - sensor may be miswired AND the whole "
              "server may now be wedged. Check with `status`; if the command "
              "port is unreachable, kill and restart Main.py.", file=sys.stderr)
        raise


def snapshot(path, timeout=DEFAULT_TIMEOUT):
    url = f"http://{HOST}:{CAMERA_PORT}/?action=snapshot"
    with urllib.request.urlopen(url, timeout=timeout) as r:
        data = r.read()
    with open(path, "wb") as f:
        f.write(data)
    return len(data)


def look_around(outdir, angles=(0, 45, 90, 135, 180)):
    """Sweeps the ULTRASONIC mount, not the camera (see module docstring) - the captured
    frames will look near-identical across angles since the fixed arm-mounted camera
    doesn't move. Useful for an ultrasonic sweep if that sensor is ever repaired, not
    for visual look-around. To actually see around the room, move the arm/chassis."""
    paths = []
    for a in angles:
        camera_pan(a)
        time.sleep(0.4)
        path = f"{outdir}/pan_{a}.jpg"
        size = snapshot(path)
        paths.append((a, path, size))
        print(f"angle {a}: {path} ({size} bytes)")
    camera_center()
    return paths


def _read_sonic_median(samples, timeout):
    """A single echo often drops out (returns 0.0) or spikes - median of a few reads
    with dropouts discarded is far steadier than any one reading."""
    vals = []
    for _ in range(samples):
        try:
            d = float(ultrasonic(timeout=timeout))
        except Exception:
            continue
        if d > 0:
            vals.append(d)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def radar_sweep(lo=0, hi=180, step=5, move_delay=0.05, read_every=3, sonic_timeout=2.5,
                samples=3, cycles=2, log=print):
    """Smoothly sweep the ultrasonic mount back and forth (fine angle steps, short delay
    between them) while periodically reading the distance, radar-style. Each reported
    reading is the median of `samples` echoes (dropouts/zeros discarded) rather than a
    single noisy echo. Returns the list of (angle, distance_cm_or_None) readings."""
    one_way = list(range(lo, hi + 1, step))
    if one_way[-1] != hi:
        one_way.append(hi)
    sweep = one_way + one_way[::-1][1:]  # lo -> hi -> lo, one full back-and-forth
    readings = []
    for cyc in range(cycles):
        for i, a in enumerate(sweep):
            camera_pan(a)
            time.sleep(move_delay)
            if i % read_every == 0:
                d = _read_sonic_median(samples, sonic_timeout)
                readings.append((a, d))
                if log:
                    log(f"[cycle {cyc+1}/{cycles}] {a:3d}°  {d if d is not None else '—'} см")
    camera_center()
    return readings


def scan_profile(angles=(30, 60, 90, 120, 150), samples=3, settle=0.3):
    """One angle->distance snapshot of what's around the car right now (ultrasonic mount
    sweep, no driving). Used as a coarse "fingerprint" of the surroundings by
    estimate_rotation_deg() to detect unwanted heading drift on a leg that has no other
    feedback (no encoders, no cv2 for the camera-based navloop.py approach)."""
    profile = {}
    for a in angles:
        camera_pan(a)
        time.sleep(settle)
        profile[a] = _read_sonic_median(samples, 2.5)
    camera_center()
    return profile


def estimate_rotation_deg(before, after, max_shift_steps=3):
    """Compare two scan_profile() readings (same angle keys) to estimate how much the
    chassis heading rotated between them, by finding the angle-index shift that best
    aligns the two distance profiles (min total abs difference over overlapping angles).

    This is sonar scan-matching, not odometry - it only works if the surroundings have
    some angular variation (a flat wall dead-on gives near-flat profiles both ways and
    won't disambiguate rotation from translation). Treat the result as a hint, not a
    measurement; cross-check with a deliberate small turn before trusting the sign.

    SIGN, calibrated 2026-07-19 against a real known LEFT steer+pulse: a LEFT turn of
    the chassis produces a NEGATIVE shift here (obstacles that used to read at a given
    mount angle now read at a lower mount angle after the car rotates left under them).
    So: positive returned degrees = chassis turned RIGHT; negative = turned LEFT.

    Returns (drift_deg, cost) where cost is the avg cm mismatch at the best alignment -
    high cost (tens of cm) means "don't trust this estimate", not "large drift".
    """
    angles = sorted(before)
    step_deg = angles[1] - angles[0] if len(angles) > 1 else 30
    before_vals = [before[a] for a in angles]
    after_vals = [after[a] for a in angles]
    n = len(angles)
    best_shift, best_cost = 0, float("inf")
    for shift in range(-max_shift_steps, max_shift_steps + 1):
        cost, count = 0.0, 0
        for i in range(n):
            j = i + shift
            if 0 <= j < n and before_vals[i] is not None and after_vals[j] is not None:
                cost += abs(before_vals[i] - after_vals[j])
                count += 1
        if count >= 3:
            avg_cost = cost / count
            if avg_cost < best_cost:
                best_cost, best_shift = avg_cost, shift
    return -best_shift * step_deg, best_cost


def move_verified(direction, mm, *, log=print):
    """drive_mm wrapped with a before/after ultrasonic scan_profile comparison, so a
    caller can detect "did this leg quietly turn the chassis" without asking the user
    and without cv2 (navloop.py's camera-based verified-straight equivalent needs cv2,
    not installed on this system as of 2026-07-19). Detection only, no auto-correction -
    steering correction for arbitrary directions/legs isn't calibrated here the way
    navloop.py's forward-only correction is.

    Returns dict: before/after profiles, estimated drift_deg (+right/-left), match cost.
    """
    before = scan_profile()
    drive_mm(direction, mm)
    time.sleep(0.3)
    after = scan_profile()
    drift_deg, cost = estimate_rotation_deg(before, after)
    if log:
        trust = "" if cost < 15 else "  (high mismatch cost - low confidence)"
        log(f"  [move_verified] {direction} {mm}mm: est. drift {drift_deg:+.0f}°"
            f" (cost {cost:.1f} cm){trust}")
    return {"before": before, "after": after, "drift_deg": drift_deg, "cost": cost}


SERVER_DIR = "/home/astra/Freenove_Three-wheeled_Smart_Car_Kit_for_Raspberry_Pi/Server"


def _server_env():
    """Main.py unconditionally builds a QApplication, so it used to need a live X
    display - which made it fail to start after every reboot once the xrdp session was
    gone (and the display number moved around, so hardcoding :10.0 didn't work either).

    Qt's `offscreen` platform plugin removes the dependency entirely: the server has no
    GUI we care about, we only want its TCP command port and mjpg-streamer. Verified
    working with no X server running at all."""
    return dict(os.environ, QT_QPA_PLATFORM="offscreen")


def find_camera_device():
    """Freenove's Start_mjpg_Streamer.sh hardcodes /dev/video0, but the USB camera does
    NOT reliably come back as video0 - unplug it and it can re-enumerate as video1 or
    video2, at which point the stream silently refuses to start. Probe for the device
    that can actually capture instead of trusting the number."""
    out = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True).stdout
    for dev in sorted(glob.glob("/dev/video*"),
                      key=lambda d: int("".join(c for c in d if c.isdigit()) or 0)):
        probe = subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats-ext"],
                               capture_output=True, text=True).stdout
        if "uvcvideo" in out and ("YUYV" in probe or "MJPG" in probe):
            return dev
    return "/dev/video0"


def restart_camera():
    """The mjpg-streamer camera can hang on its own (port 8090 times out while the
    command port still answers) - usually after the USB camera is physically moved.
    Kill just the streamer and relaunch it on whatever device the camera landed on,
    leaving Main.py alone."""
    pids = subprocess.run(["pgrep", "mjpg_streamer"], capture_output=True, text=True).stdout.split()
    if pids:
        subprocess.run(["sudo", "kill", "-9", *pids])
        time.sleep(1)
    dev = find_camera_device()
    mjpg = "/home/astra/Freenove_Three-wheeled_Smart_Car_Kit_for_Raspberry_Pi/mjpg-streamer"
    subprocess.Popen(
        ["sudo", "./mjpg_streamer",
         "-i", f"./input_uvc.so -y -d {dev} -n -r 320x240 -f 30",
         "-o", "./output_http.so -p 8090 -w ./www"],
        cwd=mjpg, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    time.sleep(3)
    print(f"camera (mjpg-streamer) restarted on {dev}")


def restart_server():
    """Full restart of Main.py (command port + camera). Needed after the many
    spontaneous Pi reboots, or when the TCP command port is wedged (e.g. an
    >Ultrasonic query with bad sensor wiring blocks the single-threaded loop)."""
    pids = subprocess.run(["pgrep", "-f", "Main.py"], capture_output=True, text=True).stdout.split()
    if pids:
        subprocess.run(["sudo", "kill", "-9", *pids])
        time.sleep(1)
    subprocess.Popen(["python", "Main.py", "-mnt"], cwd=SERVER_DIR, env=_server_env(),
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     start_new_session=True)
    time.sleep(5)
    print("server (Main.py) restarted headless (QT_QPA_PLATFORM=offscreen)")


def status():
    ok = True
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        s.connect((HOST, CMD_PORT))
        s.close()
        if port_busy():
            print(f"command port {CMD_PORT}: ЗАНЯТ другим клиентом — мои команды НЕ ДОЙДУТ "
                  f"(сервер держит только одно соединение). Закройте клиент.")
        else:
            print(f"command port {CMD_PORT}: OK (accepts connections)")
    except Exception as e:
        ok = False
        print(f"command port {CMD_PORT}: FAIL ({e})")
    try:
        with urllib.request.urlopen(f"http://{HOST}:{CAMERA_PORT}/?action=snapshot", timeout=3) as r:
            data = r.read()
        print(f"camera port {CAMERA_PORT}: OK ({len(data)} bytes)")
    except Exception as e:
        ok = False
        print(f"camera port {CAMERA_PORT}: FAIL ({e})")
    proc = subprocess.run(["pgrep", "-af", "Main.py"], capture_output=True, text=True)
    if proc.stdout.strip():
        print(f"Main.py process: {proc.stdout.strip()}")
    else:
        ok = False
        print("Main.py process: NOT RUNNING")
    return ok


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("move")
    m.add_argument("direction", choices=["forward", "backward"])
    m.add_argument("speed", type=int, help="0-100")
    m.add_argument("duration", type=float, help="seconds")

    sub.add_parser("stop")

    sp = sub.add_parser("step", help="combined steer+move+stop+snapshot in one process/connection")
    sp.add_argument("direction", choices=["forward", "backward"])
    sp.add_argument("speed", type=int, help="0-100")
    sp.add_argument("duration", type=float, help="seconds")
    sp.add_argument("path", help="where to save the post-move snapshot")
    sp.add_argument("--settle", type=float, default=0.15, help="pause after stop before snapshot")
    sp.add_argument("--steer", choices=["left", "right", "center"], default=None)
    sp.add_argument("--angle", type=int, default=40)

    st = sub.add_parser("steer")
    st.add_argument("direction", choices=["left", "right", "center"])
    st.add_argument("angle", type=int, nargs="?", default=40, help="10-60 offset from center")

    pan = sub.add_parser("pan")
    pan.add_argument("angle", type=int, help="0-180 absolute")

    tilt = sub.add_parser("tilt")
    tilt.add_argument("angle", type=int, help="0-180 absolute")

    sub.add_parser("center-camera")

    snap = sub.add_parser("snapshot")
    snap.add_argument("path")

    look = sub.add_parser("look-around")
    look.add_argument("outdir")
    look.add_argument("--angles", default="0,45,90,135,180")

    radar = sub.add_parser("radar", help="smooth back-and-forth ultrasonic sweep")
    radar.add_argument("--lo", type=int, default=0)
    radar.add_argument("--hi", type=int, default=180)
    radar.add_argument("--step", type=int, default=5)
    radar.add_argument("--move-delay", type=float, default=0.05)
    radar.add_argument("--read-every", type=int, default=3)
    radar.add_argument("--samples", type=int, default=3, help="reads per point, median-filtered")
    radar.add_argument("--cycles", type=int, default=2)

    sub.add_parser("ultrasonic")
    sub.add_parser("status")
    sub.add_parser("restart-camera", help="restart just the hung mjpg-streamer (port 8090)")
    sub.add_parser("restart-server", help="full restart of Main.py (command port + camera)")

    args = p.parse_args()

    if args.cmd == "move":
        move(args.direction, args.speed, args.duration)
    elif args.cmd == "stop":
        stop()
    elif args.cmd == "step":
        step(args.direction, args.speed, args.duration, args.path,
             settle=args.settle, steer_dir=args.steer, steer_angle=args.angle)
    elif args.cmd == "steer":
        steer(args.direction, args.angle)
    elif args.cmd == "pan":
        camera_pan(args.angle)
    elif args.cmd == "tilt":
        camera_tilt(args.angle)
    elif args.cmd == "center-camera":
        camera_center()
    elif args.cmd == "snapshot":
        size = snapshot(args.path)
        print(f"saved {args.path} ({size} bytes)")
    elif args.cmd == "look-around":
        angles = tuple(int(a) for a in args.angles.split(","))
        look_around(args.outdir, angles)
    elif args.cmd == "radar":
        radar_sweep(lo=args.lo, hi=args.hi, step=args.step, move_delay=args.move_delay,
                   read_every=args.read_every, samples=args.samples, cycles=args.cycles)
    elif args.cmd == "ultrasonic":
        print(ultrasonic())
    elif args.cmd == "status":
        ok = status()
        sys.exit(0 if ok else 1)
    elif args.cmd == "restart-camera":
        restart_camera()
        status()
    elif args.cmd == "restart-server":
        restart_server()
        status()


if __name__ == "__main__":
    main()
