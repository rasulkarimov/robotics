#!/usr/bin/env python3
"""CLI for controlling the Freenove three-wheeled car (TCP command protocol + mjpg-streamer HTTP).

Notes learned the hard way this session, baked in here so they aren't rediscovered:
- The TCP server (port 12345) handles ONE connection at a time in a single blocking loop.
  A single connect -> send -> close per call keeps things reliable.
- ">Camera Center" and ">Camera Stop" are no-ops in mTCPServer.py (literally `pass`).
  To actually center the camera, set pan/tilt to angle 90 explicitly.
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
    angle = max(0, min(180, int(angle)))
    send(f">Camera Left{angle}")


def camera_tilt(angle):
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
