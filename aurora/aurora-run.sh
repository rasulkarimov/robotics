#!/bin/bash
# Runs the Aurora930 camera pipeline as one systemd service: the SDK grabber
# (aurora_streamer) publishes frames to tmpfs, aurora_http.py serves them on
# :8090. If either dies the whole unit exits so systemd restarts it clean.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
OUTDIR=/dev/shm/aurora

mkdir -p "$OUTDIR"

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM

"$HERE/aurora_streamer" --outdir "$OUTDIR" "$@" &
pids+=($!)

# give the grabber a moment to open the device before HTTP starts answering
sleep 2

/usr/bin/python3 "$HERE/aurora_http.py" --outdir "$OUTDIR" --port 8090 &
pids+=($!)

# exit as soon as either child exits
wait -n
exit 1
