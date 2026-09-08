#!/bin/bash
# ExecStart body for claude-remote.service.
#
# --remote-control's name ("astra-robotics") is only a routing label, it does NOT
# pin a conversation. The lead/operator role needs continuity across restarts
# (reboot, Restart=on-failure, net_watchdog.restart_claude_remote), so the session
# id is pinned here and reattached every start.
#
# Why the file check instead of just `claude --resume`:
#   - `--resume <id>` when that conversation does NOT exist locally does not fail
#     cleanly in current Claude Code - it drops into the interactive resume picker
#     and blocks forever on the pty. That happened on first install (2026-09-08):
#     the unit was "active (running)" but Remote Control never finished
#     registering, so the session never showed up in the app.
#   - `--session-id <id>` refuses if a conversation with that id already exists.
# So: resume when the transcript is there, create-with-id when it is not. The id
# is stable either way and never needs re-pinning by hand.
set -u

# script(1) allocates its pty with the size of its own stdout. Under systemd that
# stdout is the journal, not a tty, so the pty comes up 0x0 - and Claude Code's
# TUI cannot render into a 0x0 terminal: it hangs in showSetupScreens() at
# startup, before Remote Control ever registers. Symptom on 2026-09-08: unit
# "active (running)", claude_remote_transcript.log empty, debug log frozen right
# after "Running showSetupScreens()", session never appeared at claude.ai/code.
# This script is script(1)'s pty child, so stty here resizes that pty before
# claude starts; COLUMNS/LINES are the belt-and-suspenders for the TUI's env fallback.
stty rows 50 cols 200 2>/dev/null || true
export COLUMNS=200 LINES=50

SESSION_ID="43d7c00d-8af3-462c-9162-0d7a73437df1"
# Claude Code stores conversations under a slug of the cwd (/ -> -).
SESSION_JSONL="$HOME/.claude/projects/-home-astra-robotics/${SESSION_ID}.jsonl"

LOG="/home/astra/robotics/claude_remote_transcript.log"
MAX_LOG_BYTES=$((20 * 1024 * 1024))

# `script -a` appends forever; this log reached 17MB in a month. Keep one
# generation so a stuck session is still diagnosable without filling the SD card.
if [ -f "$LOG" ] && [ "$(stat -c %s "$LOG")" -gt "$MAX_LOG_BYTES" ]; then
    mv -f "$LOG" "$LOG.1"
fi

CLAUDE=/home/astra/.local/bin/claude

# NO --dangerously-skip-permissions here, and no --permission-mode bypassPermissions
# either. Verified on Claude Code 2.1.263 (2026-09-08): EVERY route into Bypass
# Permissions mode shows a blocking "WARNING: ... No, exit / Yes, I accept" screen
# at startup that a human must answer with a keypress. It is NOT cached - setting
# bypassPermissionsModeAccepted true in ~/.claude.json, at top level and under
# projects["/home/astra/robotics"], did not suppress it. Headless, nothing can
# answer it, so the unit sat "active (running)" on that screen forever and Remote
# Control never registered. This is deliberate on Anthropic's side: an unattended
# permission-free agent has to be started by a person. The older Claude Code this
# unit was written against (see LEAD_NOTES 2026-08-30) did not gate it that way.
#
# Without the flag the session starts headless cleanly and Remote Control connects
# (verified: it printed its claude.ai/code session URL). Permission prompts are
# forwarded to whatever device is attached, so they get answered from the phone.
# To keep the robot autonomous for the calls it actually makes, whitelist those in
# .claude/settings.json permissions.allow rather than reaching for bypass again.
#
# --debug-file: the session runs headless, so a Remote Control handshake failure -
# the "failure notification shortly after launch" from the docs - is otherwise
# invisible. This file is where to look when the unit is "running" but the session
# never appears at claude.ai/code.
DEBUG_FILE="/home/astra/robotics/claude_remote_debug.log"
[ -f "$DEBUG_FILE" ] && [ "$(stat -c %s "$DEBUG_FILE")" -gt "$MAX_LOG_BYTES" ] && mv -f "$DEBUG_FILE" "$DEBUG_FILE.1"
COMMON=(--remote-control astra-robotics --debug-file "$DEBUG_FILE")

if [ -f "$SESSION_JSONL" ]; then
    echo "claude-remote: resuming $SESSION_ID" >&2
    "$CLAUDE" "${COMMON[@]}" --resume "$SESSION_ID"
    rc=$?
else
    echo "claude-remote: no local transcript for $SESSION_ID; creating it under that id" >&2
    "$CLAUDE" "${COMMON[@]}" --session-id "$SESSION_ID"
    rc=$?
fi

if [ "$rc" -ne 0 ]; then
    echo "claude-remote: pinned-id start failed (rc=$rc); throwaway session to stay reachable" >&2
    exec "$CLAUDE" "${COMMON[@]}"
fi
