#!/bin/bash
# ExecStart body for claude-remote.service.
#
# Without --resume, every restart of the unit (reboot, Restart=on-failure, or
# net_watchdog.restart_claude_remote) started Claude Code with an EMPTY context:
# --remote-control's name is only a routing label, it does not pin a conversation.
# The lead/operator role needs continuity across restarts, so the session id is
# pinned here.
#
# The fallback matters: if that conversation is ever gone (transcript deleted,
# home wiped) `claude --resume` exits immediately with "No conversation found",
# and with Restart=on-failure the unit would spin forever and the robot would
# have no remote session at all. Falling back to a fresh session keeps the robot
# reachable; the id below is then stale and must be re-pinned by hand.
set -u

SESSION_ID="43d7c00d-8af3-462c-9162-0d7a73437df1"
LOG="/home/astra/robotics/claude_remote_transcript.log"
MAX_LOG_BYTES=$((20 * 1024 * 1024))

# `script -a` appends forever; this log reached 17MB in a month. Keep one
# generation so a stuck session is still diagnosable without filling the SD card.
if [ -f "$LOG" ] && [ "$(stat -c %s "$LOG")" -gt "$MAX_LOG_BYTES" ]; then
    mv -f "$LOG" "$LOG.1"
fi

CLAUDE=/home/astra/.local/bin/claude

"$CLAUDE" --dangerously-skip-permissions --resume "$SESSION_ID" \
          --remote-control astra-robotics
rc=$?

if [ "$rc" -ne 0 ]; then
    echo "claude-remote: resume of $SESSION_ID failed (rc=$rc); starting fresh" >&2
    exec "$CLAUDE" --dangerously-skip-permissions --remote-control astra-robotics
fi
