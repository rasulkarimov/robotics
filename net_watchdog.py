#!/usr/bin/env python3
"""Recovers from the internet-down-while-wifi-stays-connected hang seen
2026-07-29 (see memory: internet-down-wifi-connected-hang) without needing a
manual power-cycle. That incident had wifi_state="connected" the whole time
(confirmed by the user: the internet itself was fine) while the raw
internet_state probe stayed "down" for ~10.5 hours - a Pi-local network
stack/driver wedge below the wifi L2 link, not an ISP outage. Root cause is
still unknown, so this escalates instead of guessing: a plain NetworkManager
restart first (cheap, doesn't need a reboot), and only if that fails to
restore connectivity, a full reboot - which is what manual recovery has
been so far.

It is NOT yet known whether a software reboot is actually sufficient (vs.
needing the physical power-cycle the user has used every time) - there is
no automated power-cutoff hardware for this robot. Every detection and
remediation is appended to net_watchdog_log.csv, including a note right
after boot if the previous run ended in a reboot, so that log will answer
the open question over time: if "action_reboot" is ever followed shortly by
another "hang_detected" with no intervening long uptime, software reboot
is NOT enough and the escalation needs a real power-cutoff step added.

Deliberately does its own wifi/internet probing (reusing health_log.py's
functions) rather than reading health_log.csv, so this stays useful even if
the health-log service itself is down.
"""
import csv
import os
import pwd
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from health_log import read_internet, read_wifi

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_PATH = os.path.join(BASE_DIR, "net_watchdog_log.csv")
STATE_PATH = os.path.join(BASE_DIR, "net_watchdog_state.txt")
FIELDS = ["ts", "event", "detail"]

CHECK_INTERVAL_S = 30
SOFT_RECOVERY_AFTER_S = 3 * 60    # NetworkManager restart threshold
HARD_REBOOT_AFTER_S = 10 * 60     # reboot threshold if soft recovery didn't help
POST_SOFT_ACTION_GRACE_S = 60     # let NetworkManager settle before re-probing


def log(event, detail=""):
    is_new = not os.path.exists(LOG_PATH)
    with open(LOG_PATH, "a", newline="") as f:
        w = csv.writer(f)
        if is_new:
            w.writerow(FIELDS)
        w.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"), event, detail])
        f.flush()
        os.fsync(f.fileno())
    print(",".join([time.strftime("%Y-%m-%dT%H:%M:%S"), event, detail]), flush=True)


def restart_networkmanager():
    subprocess.run(["systemctl", "restart", "NetworkManager"], timeout=30)


def reboot():
    subprocess.run(["systemctl", "reboot"])


REMOTE_USER = "astra"


def restart_claude_remote():
    # This watchdog runs as root (see net-watchdog.service), but
    # claude-remote.service is a --user unit for REMOTE_USER, so it needs
    # that user's XDG_RUNTIME_DIR to reach the right systemd --user bus.
    # Restarting on every hang recovery (not just at boot) covers the case
    # where the session was already connected and the hang itself severed
    # it without the process crashing - Restart=on-failure alone can't see
    # that, since nothing ever exits non-zero.
    # sudo sanitizes the environment on user switch, so XDG_RUNTIME_DIR has
    # to be set on the *target* command line (via `env`), not on this
    # process's own env - the latter doesn't survive the switch to astra.
    uid = pwd.getpwnam(REMOTE_USER).pw_uid
    subprocess.run(
        ["sudo", "-u", REMOTE_USER, "env", f"XDG_RUNTIME_DIR=/run/user/{uid}",
         "systemctl", "--user", "restart", "claude-remote.service"],
        timeout=30,
    )


def main():
    log("watchdog_start")

    if os.path.exists(STATE_PATH):
        with open(STATE_PATH) as f:
            pending = f.read().strip()
        os.remove(STATE_PATH)
        if pending:
            log("boot_after_action", pending)

    down_since = None
    action_taken = None  # None -> "soft" -> "hard", per incident

    while True:
        signal, wifi_state = read_wifi()
        internet_state = read_internet()
        hung = internet_state == "down" and wifi_state == "connected"

        if hung:
            if down_since is None:
                down_since = time.time()
                action_taken = None
                log("hang_detected")
            elapsed = time.time() - down_since

            if elapsed >= HARD_REBOOT_AFTER_S and action_taken != "hard":
                log("action_reboot", f"elapsed={int(elapsed)}s")
                with open(STATE_PATH, "w") as f:
                    f.write(f"rebooted after {int(elapsed)}s of hang (soft recovery already tried)")
                action_taken = "hard"
                time.sleep(2)
                reboot()
            elif elapsed >= SOFT_RECOVERY_AFTER_S and action_taken is None:
                log("action_nm_restart", f"elapsed={int(elapsed)}s")
                action_taken = "soft"
                restart_networkmanager()
                time.sleep(POST_SOFT_ACTION_GRACE_S)
                continue
        else:
            if down_since is not None:
                log("recovered", f"was_down_for={int(time.time() - down_since)}s action_taken={action_taken}")
                try:
                    restart_claude_remote()
                    log("claude_remote_restarted")
                except Exception as e:
                    log("claude_remote_restart_failed", str(e))
            down_since = None
            action_taken = None

        time.sleep(CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()
