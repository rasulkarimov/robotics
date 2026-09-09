#!/bin/bash
# The ONLY sanctioned way to move the chassis from a shell.
#
# WHY THIS EXISTS. The rule "no chassis motion without a passed preflight" was
# obeyed by hand for most of 2026-09-09 and then broken in one careless line: a
# `car.py step` written directly after `safety.py preflight` without testing its
# exit code. The gate had returned BLOCKED (38.998 cm against the 45 cm a turn
# needs) and the arc ran regardless. Nothing was hit, which is luck.
#
# A rule that depends on remembering to write `if [ $? -ne 0 ]` is not a rule.
# This makes the check structural: the move is an argument to the gate, and it
# cannot run unless the gate exits 0.
#
#   ./gated_move.sh drive -- step forward 55 0.6 /tmp/f.jpg
#   ./gated_move.sh turn --skip-human -- step forward 60 0.6 /tmp/a.jpg --steer left --angle 45
#
# --skip-human is passed straight to safety.py. It is for a SUPERVISED run only:
# it does not count toward any ladder criterion, and the caller must say in the
# log that a human was watching.
set -uo pipefail
REPO="$(cd "$(dirname "$0")" && pwd)"
ACTION="${1:-}"; shift || true
case "$ACTION" in drive|turn) ;; *) echo "usage: $0 {drive|turn} [--skip-human] -- <car.py args...>" >&2; exit 64;; esac
GATE_ARGS=()
while [ $# -gt 0 ] && [ "$1" != "--" ]; do GATE_ARGS+=("$1"); shift; done
[ "${1:-}" = "--" ] || { echo "missing -- separating gate args from the move" >&2; exit 64; }
shift
[ $# -gt 0 ] || { echo "no move given after --" >&2; exit 64; }

python3 "$REPO/safety.py" preflight "$ACTION" "${GATE_ARGS[@]+"${GATE_ARGS[@]}"}" >/tmp/gate.json 2>&1
rc=$?
verdict=$(python3 -c "import json;print(json.load(open('/tmp/gate.json')).get('verdict','?'))" 2>/dev/null || echo "?")
if [ $rc -ne 0 ]; then
    echo "REFUSED: preflight $ACTION -> $verdict (exit $rc). Move NOT executed."
    python3 -c "import json;d=json.load(open('/tmp/gate.json'));print('  blocks :',d.get('blocks'));print('  unknown:',d.get('unknown'))" 2>/dev/null
    exit $rc
fi
echo "gate $ACTION -> $verdict, executing: car.py $*"
exec python3 "$REPO/car.py" "$@"
