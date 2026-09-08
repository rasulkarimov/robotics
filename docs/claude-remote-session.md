# The persistent Claude remote session — three ways it silently hung

`claude-remote.service` gives the robot a Claude Code session that lives on the
Pi and is reachable from the Claude app / claude.ai/code. Reinstalled and fixed
2026-09-08 on Claude Code **2.1.263**.

The unit and its body (`systemd/claude-remote.service`,
`systemd/claude-remote-run.sh`) were already in this repo and had worked before
(LEAD_NOTES 2026-08-30). On a fresh SD card, against a current Claude Code, they
did not — and every failure looked identical from the outside: `systemctl --user
status` said **`active (running)`** while nothing ever appeared in the app. Three
separate causes, found in this order.

## The tell: `.key` without `.json`

`~/.claude/sessions/` gets two files per session. A `<pid>.<hash>.key` appears
early, as soon as the process starts. A **`<pid>.json`** — carrying
`bridgeSessionId`, `sessionId`, `status` — appears only once Remote Control has
actually registered.

**`.key` alone means the process started and never connected.** That single fact
would have saved most of the time spent here. Check it first:

```bash
ls ~/.claude/sessions/                       # want <pid>.json, not just <pid>.key
ss -tnp | grep pid=<pid>                     # want many ESTAB to Anthropic
```

Note that `journalctl --user -u claude-remote` is **always empty** on this box:
user journald does not persist ("No journal files were found"). The logs that do
exist are `claude_remote_transcript.log` (the pty, via `script`) and
`claude_remote_debug.log` (`--debug-file`, added for exactly this reason).

## 1. `script(1)` hands Claude a 0×0 terminal

`script` sizes its pty from its own stdout. Under systemd that is the journal,
not a tty, so the pty comes up **0 rows × 0 columns**. Claude Code's TUI cannot
render into that: it hangs inside `showSetupScreens()` at startup, before Remote
Control is ever reached. Symptom is a completely **empty** transcript log and a
debug log frozen right after `[STARTUP] Running showSetupScreens()...`, with the
process alive at a few percent CPU in `epoll_wait`.

Fix, at the top of `claude-remote-run.sh` (which is `script`'s pty child, so it
can resize the pty it was handed):

```bash
stty rows 50 cols 200 2>/dev/null || true
export COLUMNS=200 LINES=50
```

## 2. `--resume` of a conversation that does not exist blocks on the picker

The run script pins a session id so a restart keeps its context. But
`claude --resume <id>` for an id with no local `.jsonl` does **not** exit with
"No conversation found" any more — it drops into the interactive resume picker
and waits forever. On a fresh SD card that is every first start.

The script now decides by looking, instead of relying on failure semantics:

```bash
SESSION_JSONL="$HOME/.claude/projects/-home-astra-robotics/${SESSION_ID}.jsonl"
[ -f "$SESSION_JSONL" ] && use --resume "$SESSION_ID" || use --session-id "$SESSION_ID"
```

`--session-id` creates the conversation under that exact id, so the pin is
self-healing: the next restart finds the `.jsonl` and resumes. The id never needs
re-pinning by hand.

## 3. Bypass Permissions mode cannot run headless at all

This is the important one, and it is a deliberate change on Anthropic's side.

On 2.1.263, **every** route into Bypass Permissions mode — both
`--dangerously-skip-permissions` and `--permission-mode bypassPermissions` —
shows a blocking screen at startup:

```
WARNING: Claude Code running in Bypass Permissions mode
  No, exit
  Yes, I accept          <- needs a human keypress
```

And it is **not cached**. Setting `bypassPermissionsModeAccepted: true` in
`~/.claude.json`, at top level *and* under `projects["/home/astra/robotics"]`,
did not suppress it. Headless there is nobody to press a key, so the unit sits on
that screen indefinitely and Remote Control never registers.

An unattended, permission-free agent now has to be started by a person. The older
Claude Code this unit was originally written against did not gate it this way.

**So the service runs with no bypass flag at all**, and that works: the session
starts headless and Remote Control connects, printing its `claude.ai/code`
session URL. Permission prompts are forwarded to whatever device is attached and
answered from the phone.

To keep the robot autonomous for the calls it actually makes, the answer is
`.claude/settings.json` → `permissions.allow`, not reaching for bypass again.

## Persistence

| failure | what covers it |
| --- | --- |
| process crash | `Restart=on-failure`, `RestartSec=10` |
| reboot | `WantedBy=default.target` + **linger** enabled for `astra`, so it starts with no login |
| network wedge | `ExecStartPre=wait_for_internet_gate.py` (refuses to start into a hang) + `net_watchdog.restart_claude_remote()` |
| conversation memory | the pinned session id, with the self-healing fallback above |

`net_watchdog.py` is not itself installed as a service on this machine yet, so
that last row is only half-wired.

## Operating notes

- Install is a `--user` unit (`~/.config/systemd/user/claude-remote.service`);
  `net_watchdog` restarts it via `systemctl --user`, so do not move it to a
  system unit without changing that.
- If `systemctl --user restart` is unavailable, `kill -9` on the unit's MainPID
  (the `script` process) plus the `claude` pid is an equivalent restart —
  `Restart=on-failure` picks it up.
- Raspberry Pi Connect is installed and signed in here, with remote shell
  allowed: <https://connect.raspberrypi.com> gives a browser shell, including
  from a phone, when SSH is not available. That is the way to answer any
  interactive Claude Code screen on this machine.
- The session's name in the app is auto-generated (e.g. `robotics-4d`) unless
  set; `/rename astra-robotics` inside the session fixes it.
