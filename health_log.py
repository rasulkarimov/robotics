#!/usr/bin/env python3
"""Continuous, crash-surviving telemetry: power/thermal throttle state + wifi link.

Exists because a Pi that loses power (brownout under motor/servo load - see the
single-shared-battery memory) or drops wifi just vanishes mid-session with nothing
in bash history or the journal to explain it (journald here had zero history from
before the current boot when this was written, 2026-07-27 - Storage was never made
explicit, see /etc/systemd/journald.conf.d/persistent.conf). This does NOT open the
arm's USB HID connection (arm.getBatteryVoltage() requires exclusive access and this
must never contend with a live grasp session) - it only reads the Pi's own firmware
mailbox (vcgencmd) and wifi link state, both safe to poll continuously.

A timestamp gap in health_log.csv IS the crash evidence: compare the gap to
`journalctl --list-boots` after the fact. throttled bits are cumulative-since-boot
for the high nibble (0x1_0000 = under-voltage occurred at some point this boot), so
a nonzero value here confirms brownout even if it recovered before you looked.
"""
import csv
import os
import re
import subprocess
import time

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "health_log.csv")
INTERVAL_S = 5
FIELDS = ["ts", "throttled", "undervolt_now", "undervolt_ever", "temp_c", "wifi_signal_dbm", "wifi_state"]


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


def main():
    is_new = not os.path.exists(LOG_PATH)
    f = open(LOG_PATH, "a", newline="")
    writer = csv.writer(f)
    if is_new:
        writer.writerow(FIELDS)
        f.flush()
        os.fsync(f.fileno())

    while True:
        throttled, undervolt_now, undervolt_ever = read_throttled()
        temp = read_temp()
        signal, wifi_state = read_wifi()
        writer.writerow([
            time.strftime("%Y-%m-%dT%H:%M:%S"),
            hex(throttled) if throttled is not None else "",
            undervolt_now if undervolt_now is not None else "",
            undervolt_ever if undervolt_ever is not None else "",
            temp if temp is not None else "",
            signal if signal is not None else "",
            wifi_state,
        ])
        f.flush()
        os.fsync(f.fileno())
        time.sleep(INTERVAL_S)


if __name__ == "__main__":
    main()
