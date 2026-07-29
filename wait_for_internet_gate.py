#!/usr/bin/env python3
"""ExecStartPre gate for claude-remote.service.

Starting the remote-control session the moment NetworkManager reports
wifi "connected" isn't enough - see the internet-down-wifi-connected-hang
memory: wifi can stay associated for hours while the Pi has no real route
out. That's exactly what happened on 2026-07-29: the service started at
20:00:58, during a hang that ran 19:58:24-20:03:59, and sat stuck with a
dead connection for 44 minutes because nothing ever restarted it.

Exits 1 when there's no real internet yet, so systemd's Restart=on-failure
+ RestartSec=10 on the unit turns this into a retry-until-really-online
loop for free, instead of starting into a hang and staying broken.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from health_log import read_internet

sys.exit(0 if read_internet() == "up" else 1)
