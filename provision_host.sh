#!/bin/bash
# Rebuild everything this robot needs that does NOT live in the git tree.
#
# WHY THIS FILE EXISTS. Host-side setup has now been silently destroyed several
# times: the venv, the mjpg-streamer build, whisper.cpp (three times), and on
# 2026-09-08 an SD-card rebuild that took out every systemd unit, both journald
# and NetworkManager dropins, and the health log. Each time it was rebuilt by
# hand from memory, and each time something was missed - net-watchdog.service had
# only ever existed in /etc and vanished with no copy anywhere.
#
# Everything below is idempotent. Run it after any reflash, and run it when
# something is behaving as though a service is missing, because it probably is.
#
#   ./provision_host.sh            # everything
#   ./provision_host.sh --check    # report what is missing, change nothing
set -uo pipefail
REPO="$(cd "$(dirname "$0")" && pwd)"
CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

ok=0; missing=0
say()  { printf '  %-34s %s\n' "$1" "$2"; }
need() { missing=$((missing+1)); say "$1" "MISSING${2:+ - $2}"; }
have() { ok=$((ok+1));           say "$1" "ok"; }

run() { if [ "$CHECK" = 1 ]; then return 0; fi; "$@"; }

# Do NOT pipe systemctl into `grep -q` here. grep -q exits on the first match and
# closes the pipe, systemctl takes SIGPIPE, and `pipefail` then reports the whole
# pipeline as failed even though the match succeeded - so an installed unit gets
# reported MISSING, nondeterministically, depending on who finishes first. Caught
# live: car-server.service (installed and active) was reported missing while
# net-watchdog right after it was reported ok. Command substitution has no pipe
# and no race.
unit_installed()      { [[ "$(systemctl list-unit-files "$1.service" 2>/dev/null)" == *"$1"* ]]; }
user_unit_installed() { [[ "$(systemctl --user list-unit-files "$1.service" 2>/dev/null)" == *"$1"* ]]; }

echo "== system units =="
for u in car-server net-watchdog health-log aurora-camera; do
    if unit_installed "$u"; then
        have "$u.service"
    else
        need "$u.service"
        run sudo cp "$REPO/systemd/$u.service" /etc/systemd/system/
        run sudo systemctl daemon-reload
        run sudo systemctl enable --now "$u.service"
    fi
done

echo "== user units (need linger, which is already on for astra) =="
# sound-watch is only installable if there is something to listen with. The
# microphone lived on the old UVC webcam; the Aurora930 that replaced it carries
# none, so on 2026-09-09 the unit restart-looped every 10 s against a device that
# was not there. Do not install what cannot work.
USER_UNITS="camera-watch claude-remote"
if [ -n "$(arecord -l 2>/dev/null | grep '^card ' || true)" ]; then
    USER_UNITS="$USER_UNITS sound-watch"
else
    say "sound-watch.service (--user)" "SKIPPED - no capture device (no microphone)"
fi
for u in $USER_UNITS; do
    if user_unit_installed "$u"; then
        have "$u.service (--user)"
    else
        need "$u.service (--user)"
        run mkdir -p "$HOME/.config/systemd/user"
        run cp "$REPO/systemd/$u.service" "$HOME/.config/systemd/user/"
        run systemctl --user daemon-reload
        run systemctl --user enable --now "$u.service"
    fi
done

echo "== dropins =="
if [ -f /etc/systemd/journald.conf.d/persistent.conf ]; then
    have "journald persistent storage"
else
    need "journald persistent storage" "logs die on every reboot"
    run sudo mkdir -p /etc/systemd/journald.conf.d
    run sudo cp "$REPO/systemd/journald-persistent.conf" \
        /etc/systemd/journald.conf.d/persistent.conf
    run sudo mkdir -p /var/log/journal
    run sudo systemctl restart systemd-journald
    # The restart alone is NOT enough. systemd-journal-flush.service is `static`
    # and already ran at boot, before this dropin existed, so it will not re-run
    # and journald keeps writing to /run. Verified live: only an explicit flush
    # created /var/log/journal/<machine-id>/.
    run sudo journalctl --flush
fi
if [ -f /etc/NetworkManager/conf.d/99-wifi-powersave-off.conf ]; then
    have "wifi power-save off"
else
    need "wifi power-save off" "suspected cause of SSH drops"
    if [ "$CHECK" = 0 ]; then
        sudo mkdir -p /etc/NetworkManager/conf.d
        printf '[connection]\nwifi.powersave = 2\n' \
            | sudo tee /etc/NetworkManager/conf.d/99-wifi-powersave-off.conf >/dev/null
        sudo systemctl reload NetworkManager
    fi
fi

echo "== arm toolchain =="
if [ -x /home/astra/tools/venv/bin/python3 ]; then
    have "arm venv"
else
    need "arm venv" "arm.py cannot run at all without it"
    run python3 -m venv /home/astra/tools/venv
    run /home/astra/tools/venv/bin/pip install -q -r "$REPO/requirements.txt"
fi

echo "== camera (Aurora930) =="
if [ -f /etc/udev/rules.d/99-deptrum-libusb.rules ]; then
    have "deptrum udev rule"
else
    need "deptrum udev rule" "camera needs root without it"
    run bash "$REPO/aurora/provision_aurora.sh"
fi
if [ -x "$REPO/aurora/aurora_streamer" ]; then
    have "aurora_streamer built"
else
    need "aurora_streamer built"
    run make -C "$REPO/aurora" all
fi

echo "== i2c (the chassis motor/sensor shield) =="
if [ -e /dev/i2c-1 ]; then
    have "/dev/i2c-1"
else
    need "/dev/i2c-1" "Main.py dies on smbus.SMBus(1); NEEDS A REBOOT after enabling"
    run sudo raspi-config nonint do_i2c 0
fi

echo "== speech =="
if [ -x /home/astra/tools/whisper.cpp/build/bin/whisper-cli ] \
   || [ -x /home/astra/tools/whisper.cpp/main ]; then
    have "whisper.cpp"
else
    need "whisper.cpp" "robot can notice a noise but not hear words"
    echo "    (run $REPO/provision_whisper.sh - it is slow, so it is not automatic)"
fi

echo
echo "$ok present, $missing missing."
[ "$CHECK" = 1 ] && echo "--check: nothing was changed."
exit 0
