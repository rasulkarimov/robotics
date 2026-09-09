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
case "$ACTION" in drive|turn|nudge) ;; *) echo "usage: $0 {drive|nudge|turn} [--skip-human] -- <car.py args...>" >&2; exit 64;; esac
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
# STEERING IS REQUIRED, NOT DEFAULTED.
# car.py step leaves the front wheels wherever the previous move put them when no
# --steer is given, so every "straight" move issued after an arc silently curves.
# That is how a 5 cm correction wandered sideways all evening.
#
# This first auto-appended `--steer center`. The user asked for something
# stronger: "Хорошо бы сделать это обязательным параметром." They are right - a
# silent default still lets the caller not think about it, and the whole failure
# was not thinking about it. Now the move is REFUSED unless the steering is
# stated, so every drive carries a deliberate decision about where the wheels
# point.
case " $* " in
    *" --steer "*) ;;
    *"step"*)
        echo "REFUSED: --steer is REQUIRED for a step. State it every time:" >&2
        echo "  --steer center      straight" >&2
        echo "  --steer left|right --angle N   an arc (N is clamped to STEER_MAX_SAFE)" >&2
        echo "The wheels keep their last angle otherwise, so an unstated steer is" >&2
        echo "whatever the previous move left behind - which is how straight moves curve." >&2
        exit 64 ;;
esac
echo "gate $ACTION -> $verdict, executing: car.py $*"
exec python3 "$REPO/car.py" "$@"
