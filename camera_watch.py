#!/usr/bin/env python3
"""Bring the camera back when it dies - and leave it alone when it is asleep.

The camera failed four times on 2026-08-30 (00:08, 10:36, 11:08, 12:12), always
the same shape: `Main.py` alive, command port answering, `mjpg-streamer` dead.
`car-server.service` restarts the server process and never checks that the
streamer bound its port, so "alive but blind" is this robot's normal failure
mode and nothing noticed it.

A watchdog was proposed once before and withdrawn, because one of those outages
turned out to be `car.py sleep` - deliberate power saving on a battery the Pi
shares - and a naive watchdog would have fought every nap the robot ever took.
That is now decidable: `sleep` leaves a marker, so this only acts when the
camera is down and NO marker is present.

    camera_watch.py check     # one look, print the verdict, change nothing
    camera_watch.py watch     # the daemon
"""
import argparse
import csv
import os
import socket
import subprocess
import sys
import time
import urllib.request

REPO = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(REPO, "camera_log.csv")
SLEEP_MARKER = os.path.join(REPO, ".asleep")

HOST, CAMERA_PORT = "127.0.0.1", 8090
INTERVAL_S = 30.0
# Two consecutive misses before acting: a single failed poll can be the streamer
# mid-restart, or an orient sweep holding the device.
CONFIRM = 2
# If it keeps dying this often, restarting it again is not the answer and
# something needs a person. Spinning would just hide the real fault.
MAX_RESTARTS_PER_HOUR = 6


def camera_up(timeout=3):
    try:
        with urllib.request.urlopen(
                f"http://{HOST}:{CAMERA_PORT}/?action=snapshot", timeout=timeout) as r:
            return len(r.read()) > 0
    except Exception:
        return False


def asleep():
    try:
        with open(SLEEP_MARKER) as f:
            return f.read().strip() or "unknown time"
    except FileNotFoundError:
        return None


def log(action, detail=""):
    new = not os.path.exists(LOG)
    with open(LOG, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["ts", "action", "detail"])
        w.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"), action, detail])


def cmd_check(_args):
    up = camera_up()
    nap = asleep()
    if up:
        print("camera: UP")
    elif nap:
        print(f"camera: off, but the robot is ASLEEP since {nap} - leaving it alone")
    else:
        print("camera: DOWN and not asleep - a watchdog would restart it")
    return 0


def cmd_watch(args):
    misses = 0
    restarts = []
    print(f"watching port {CAMERA_PORT} every {INTERVAL_S:.0f}s "
          f"(confirm {CONFIRM}, max {MAX_RESTARTS_PER_HOUR}/h)", flush=True)
    while True:
        if camera_up():
            misses = 0
            time.sleep(INTERVAL_S)
            continue

        nap = asleep()
        if nap:
            # Deliberate. Not our business.
            misses = 0
            time.sleep(INTERVAL_S)
            continue

        misses += 1
        if misses < CONFIRM:
            time.sleep(INTERVAL_S)
            continue

        now = time.time()
        restarts = [t for t in restarts if now - t < 3600]
        if len(restarts) >= MAX_RESTARTS_PER_HOUR:
            log("giving_up", f"{len(restarts)} restarts in the last hour - needs a person")
            print("too many restarts this hour; not trying again", flush=True)
            time.sleep(INTERVAL_S * 10)
            continue

        log("restarting", f"down for {misses} consecutive checks")
        r = subprocess.run(["python3", "car.py", "restart-camera"], cwd=REPO,
                           capture_output=True, text=True, timeout=120)
        time.sleep(3)
        ok = camera_up()
        restarts.append(now)
        misses = 0
        log("restarted" if ok else "restart_failed",
            (r.stdout or r.stderr).strip().splitlines()[-1][:120] if (r.stdout or r.stderr) else "")
        print(f"camera was down -> restart {'ok' if ok else 'FAILED'}", flush=True)
        time.sleep(INTERVAL_S)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="one look, change nothing")
    sub.add_parser("watch", help="run as a daemon")
    args = ap.parse_args()
    return {"check": cmd_check, "watch": cmd_watch}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main() or 0)
