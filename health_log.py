#!/usr/bin/env python3
"""Continuous, crash-surviving telemetry: power/thermal throttle state + wifi link.

Exists because a Pi that loses power (brownout under motor/servo load - see the
single-shared-battery memory) or drops wifi just vanishes mid-session with nothing
in bash history or the journal to explain it (journald here had zero history from
before the current boot when this was written, 2026-07-27 - Storage was never made
explicit, see /etc/systemd/journald.conf.d/persistent.conf). This does NOT open the
arm's USB HID connection (arm.getBatteryVoltage() requires exclusive access and this
must never contend with a live grasp session) - it only reads the Pi's own firmware
mailbox (vcgencmd), wifi link state, and a raw-IP internet reachability probe,
all safe to poll continuously.

A timestamp gap in health_log.csv IS the crash evidence: compare the gap to
`journalctl --list-boots` after the fact. throttled bits are cumulative-since-boot
for the high nibble (0x1_0000 = under-voltage occurred at some point this boot), so
a nonzero value here confirms brownout even if it recovered before you looked.

2026-07-28 postmortem: a hard power-loss reboot hit and health_log.csv itself did
NOT survive it - the file's birth time on the new boot was the new boot's start,
meaning every pre-crash row (hours of it) was gone despite flush()+fsync(fileno())
on every write. That fsync only durs the file's *data*, never the parent directory's
entry for it - on an unclean shutdown the whole file can vanish in ext4 journal
recovery even though each write was fsynced. Fix: fsync the directory right after
creating the file, AND print() every row too - this process is a systemd service
with default StandardOutput=journal, so each row also lands in
`journalctl -u health-log.service`, a completely separate storage path (proven to
survive this exact crash already) from the CSV file. If the CSV ever vanishes again,
the journal is the fallback.
"""
import csv
import os
import re
import socket
import subprocess
import time

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "health_log.csv")
INTERVAL_S = 5
INTERNET_PROBE_ADDR = ("1.1.1.1", 443)
FIELDS = [
    "ts",
    "throttled",
    "undervolt_now",
    "undervolt_ever",
    "temp_c",
    "wifi_signal_dbm",
    "wifi_state",
    "internet_state",
]


def read_throttled():
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2).stdout
        val = int(out.strip().split("=")[1], 16)
    except Exception:
        return None, None, None
    return val, bool(val & 0x1), bool(val & 0x10000)


def read_temp():
    try:
        out = subprocess.run(["vcgencmd", "measure_temp"], capture_output=True, text=True, timeout=2).stdout
        return float(re.search(r"[\d.]+", out).group())
    except Exception:
        return None


def read_wifi():
    try:
        out = subprocess.run(["/sbin/iw", "dev", "wlan0", "link"], capture_output=True, text=True, timeout=2).stdout
        if out.startswith("Not connected"):
            return None, "disconnected"
        m = re.search(r"signal:\s*(-?\d+)\s*dBm", out)
        return (int(m.group(1)) if m else None), "connected"
    except Exception:
        return None, "unknown"


def read_internet():
    # wifi_state above only proves L2 association with the AP - a router/ISP
    # outage still shows "connected" there while every cloud-dependent thing
    # (claude-remote's tunnel, rpi-connect's relay) is unreachable for hours.
    # Raw IP:port connect, no DNS involved, so this can't be confused with a
    # DNS-only failure.
    try:
        with socket.create_connection(INTERNET_PROBE_ADDR, timeout=2):
            return "up"
    except OSError:
        return "down"


def main():
    is_new = not os.path.exists(LOG_PATH)
    f = open(LOG_PATH, "a", newline="")
    writer = csv.writer(f)
    if is_new:
        writer.writerow(FIELDS)
        f.flush()
        os.fsync(f.fileno())
        # fsync the file's data, not its directory entry - without this the
        # entry can be lost on an unclean shutdown even though every write
        # to the file itself was fsynced (see 2026-07-28 postmortem above).
        dir_fd = os.open(os.path.dirname(LOG_PATH), os.O_RDONLY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)

    while True:
        throttled, undervolt_now, undervolt_ever = read_throttled()
        temp = read_temp()
        signal, wifi_state = read_wifi()
        internet_state = read_internet()
        row = [
            time.strftime("%Y-%m-%dT%H:%M:%S"),
            hex(throttled) if throttled is not None else "",
            undervolt_now if undervolt_now is not None else "",
            undervolt_ever if undervolt_ever is not None else "",
            temp if temp is not None else "",
            signal if signal is not None else "",
            wifi_state,
            internet_state,
        ]
        writer.writerow(row)
        f.flush()
        os.fsync(f.fileno())
        # Second, independent durability path: journald has already survived
        # a crash that erased this CSV once. StandardOutput=journal (the
        # systemd default) picks this up automatically.
        print(",".join(str(x) for x in row), flush=True)
        time.sleep(INTERVAL_S)


if __name__ == "__main__":
    main()
