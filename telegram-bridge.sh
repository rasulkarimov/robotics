#!/bin/bash
# Claude Code <-> Telegram bridge (@astra3001_bot).
#
# The bridge is not a daemon: it is an MCP server that only exists inside a
# running Claude Code session, so this script keeps such a session alive.
# `script` allocates the pty the interactive TUI needs -- systemd gives us none.
set -uo pipefail

# bun (MCP server runtime) is at /usr/local/bin; claude is in ~/.local/bin.
# systemd user services start with a minimal PATH, so spell it out.
export PATH="/usr/local/bin:/usr/bin:/bin:/home/astra/.local/bin"
export TERM=xterm-256color

cd /home/astra/Freenove_Three-wheeled_Smart_Car_Kit_for_Raspberry_Pi || exit 1

# Re-apply the inbound re-delivery patch if a plugin re-extract reverted it.
# The patch adds a server-side queue that re-delivers inbound messages missed
# while the session was rate-limited or rebooting; it lives in the versioned
# plugin cache, so a re-extract would silently drop it. Self-heal ONLY when the
# live file is the exact known-base version -- if it changed (a real upstream
# update), warn and run unpatched so the change is noticed and re-patched.
patch_dir=$(ls -d "$HOME"/.claude/plugins/cache/claude-plugins-official/telegram/*/ 2>/dev/null | sort -V | tail -1)
live="${patch_dir}server.ts"
patched="$HOME/tools/telegram-server.patched.ts"
base_sha_file="$HOME/tools/telegram-server.base.sha256"
if [[ -n $patch_dir && -f $live && -f $patched && -f $base_sha_file ]]; then
  if ! grep -q 'astra patch: inbound re-delivery' "$live"; then
    live_sha=$(sha256sum "$live" | awk '{print $1}')
    base_sha=$(cat "$base_sha_file")
    if [[ $live_sha == "$base_sha" ]]; then
      cp "$patched" "$live" && echo "telegram-bridge: re-applied inbound re-delivery patch to $live" >&2
    else
      echo "telegram-bridge: WARNING $live differs from known base (plugin update?); re-delivery patch NOT applied" >&2
    fi
  fi
fi

exec /usr/bin/script -qec \
  "claude --channels plugin:telegram@claude-plugins-official \
          --permission-mode bypassPermissions \
          --add-dir /home/astra/tools" \
  /dev/null
