#!/bin/bash
# Build whisper.cpp and fetch a model, so the robot can hear WHAT was said.
#
# This exists because the same thing has now gone missing three times on this
# box: the arm venv, the mjpg-streamer binary, and whisper.cpp itself. Anything
# that is compiled or downloaded lives outside git, so a rebuild wipes it
# silently and the failure looks like a bug rather than an absent install.
# `stt.sh` is committed and correct; it just had nothing to run.
#
# Idempotent: re-running skips what is already in place.
#
#   ./provision_whisper.sh              # base-q5_1, the recommended default
#   MODEL=tiny-q5_1 ./provision_whisper.sh
set -euo pipefail

SRC=/home/astra/whisper.cpp
MODELS=/home/astra/whisper-models
# Russian speech, so the model must be MULTILINGUAL - the .en variants are out.
# base-q5_1 is the balance point on a Pi 4: clearly better Russian than tiny,
# and quantised it is smaller (~57 MB) than tiny's f16 (75 MB). `small` is far
# better again and far too slow on four Cortex-A72 cores for a robot that is
# supposed to react.
MODEL="${MODEL:-base-q5_1}"
MODEL_FILE="$MODELS/ggml-${MODEL}.bin"
URL="https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-${MODEL}.bin"

echo "== whisper.cpp source"
if [[ -d $SRC/.git ]]; then
    echo "   already cloned: $SRC"
else
    git clone --depth 1 https://github.com/ggml-org/whisper.cpp "$SRC"
fi

echo "== build"
BIN="$SRC/build/bin/whisper-cli"
if [[ -x $BIN ]]; then
    echo "   already built: $BIN"
else
    # Four cores, and the arm and the Pi share one battery - do this on charge.
    cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build "$SRC/build" -j4 --config Release
fi
[[ -x $BIN ]] || { echo "build produced no whisper-cli" >&2; exit 1; }

echo "== model $MODEL"
mkdir -p "$MODELS"
if [[ -s $MODEL_FILE ]]; then
    echo "   already present: $MODEL_FILE ($(du -h "$MODEL_FILE" | cut -f1))"
else
    curl -fL --progress-bar -o "$MODEL_FILE.part" "$URL"
    mv "$MODEL_FILE.part" "$MODEL_FILE"
    echo "   downloaded: $(du -h "$MODEL_FILE" | cut -f1)"
fi

echo
echo "done. Point stt.sh at it with:"
echo "   export WHISPER_MODEL=$MODEL_FILE"
echo "Verify on real audio (record a few seconds first):"
echo "   arecord -D plughw:3,0 -f S16_LE -r 16000 -c 1 -d 5 /tmp/say.wav"
echo "   WHISPER_MODEL=$MODEL_FILE ./stt.sh /tmp/say.wav ru"
