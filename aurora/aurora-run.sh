#!/bin/bash
# Runs the Aurora930 camera pipeline as one systemd service: the SDK grabber
# (aurora_streamer) publishes frames to tmpfs, aurora_http.py serves them on
# :8090. If either dies the whole unit exits so systemd restarts it clean.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
OUTDIR=/dev/shm/aurora

mkdir -p "$OUTDIR"

pids=()
# Distinguish "someone stopped us" from "a child died on its own". Without this
# an explicit `systemctl stop` - which is what `car.py sleep` does to save the
# shared battery - left the unit in state `failed`, so a deliberate nap looked
# exactly like a crashed camera to anyone reading systemctl the next morning.
terminating=0
cleanup() { kill "${pids[@]}" 2>/dev/null; wait 2>/dev/null; }
on_term() { terminating=1; cleanup; exit 0; }
trap cleanup EXIT
trap on_term INT TERM

"$HERE/aurora_streamer" --outdir "$OUTDIR" "$@" &
pids+=($!)

# give the grabber a moment to open the device before HTTP starts answering
sleep 2

/usr/bin/python3 "$HERE/aurora_http.py" --outdir "$OUTDIR" --port 8090 &
pids+=($!)

# exit as soon as either child exits - non-zero, so systemd restarts the pair
wait -n
[ "$terminating" = 1 ] && exit 0
exit 1
