#!/bin/bash
# Watchdog for the Claude Code <-> Telegram bridge (@astra3001_bot).
#
# Telegram allows exactly one getUpdates consumer per bot token. The plugin's MCP
# server enforces that by SIGTERM-ing whoever holds bot.pid when it starts -- and
# since the plugin is enabled globally, every ordinary `claude` session on this Pi
# starts one and hijacks the bot from the bridge. Such a session has no --channels,
# so it never answers: it just drains the updates and the bot goes silent while the
# unit still reads "active". Restart the bridge so it takes the bot back.
set -uo pipefail

UNIT=claude-telegram.service
PID_FILE="$HOME/.claude/channels/telegram/bot.pid"
GRACE_US=45000000

if [[ $(systemctl --user is-active "$UNIT") != active ]]; then
  echo "bridge not active -- starting"
  exec systemctl --user restart "$UNIT"
fi

# A freshly started bridge needs ~10s to boot its MCP server. Don't judge it before that.
since=$(systemctl --user show -p ActiveEnterTimestampMonotonic --value "$UNIT")
now=$(awk '{printf "%d", $1 * 1000000}' /proc/uptime)
(( now - since < GRACE_US )) && exit 0

main=$(systemctl --user show -p MainPID --value "$UNIT")
owner=$(cat "$PID_FILE" 2>/dev/null) || owner=""

# The live poller must be a descendant of the bridge session, not of some other claude.
owned=0
if [[ -n $owner && -d /proc/$owner ]]; then
  p=$owner
  while [[ -n $p && $p != 1 && $p != 0 ]]; do
    [[ $p == "$main" ]] && { owned=1; break; }
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
  done
fi

(( owned )) && exit 0

echo "bridge lost the bot (bot.pid=${owner:-none}, bridge MainPID=$main) -- restarting"
exec systemctl --user restart "$UNIT"
