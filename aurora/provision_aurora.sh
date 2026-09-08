#!/bin/bash
# Idempotent setup for the Aurora930 camera pipeline. Safe to re-run; meant to be
# run after an SD-card rebuild (see memory: reflash-lost-provisioning-2026-09).
#
# Assumes the Deptrum SDK is already unpacked at $SDK (it survives a reflash only
# if it was put outside the wiped area — it currently lives in ~/tools).
set -euo pipefail

SDK=/home/astra/tools/deptrum-aurora930/sdk
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "== apt deps =="
sudo apt-get update -qq
sudo apt-get install -y libusb-1.0-0 libturbojpeg0-dev build-essential

echo "== udev rule (device node 0666 for non-root libusb) =="
sudo cp "$SDK/scripts/99-deptrum-libusb.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "== build =="
[ -e "$SDK/lib/libdeptrum_stream_aurora900.so" ] || { echo "SDK not found at $SDK"; exit 1; }
make -C "$HERE" clean all

echo "== systemd unit =="
sudo cp "$HERE/../systemd/aurora-camera.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now aurora-camera.service

echo "== verify =="
sleep 5
curl -fsS -o /dev/null -w "snapshot: http %{http_code}\n" "http://127.0.0.1:8090/?action=snapshot"
systemctl is-active aurora-camera.service
echo "done"
