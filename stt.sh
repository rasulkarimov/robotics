#!/bin/bash
# Speech-to-text for Telegram voice messages: prints the transcript to stdout.
#
# Claude has no audio modality. The telegram bridge forwards a voice message and can
# download the file, but the model cannot listen to it -- so voice only works if
# something turns it into text first. That is this script.
#
#   stt.sh <audio-file> [lang]     # lang defaults to ru, e.g. `stt.sh voice.oga en`
set -euo pipefail

MODEL="${WHISPER_MODEL:-/home/astra/whisper-models/ggml-tiny.bin}"
BIN=/home/astra/whisper.cpp/build/bin/whisper-cli
[[ -x $BIN ]] || BIN=/home/astra/whisper.cpp/build/bin/main

(( $# >= 1 )) || { echo "usage: stt.sh <audio-file> [lang]" >&2; exit 2; }
IN=$1
LANG_="${2:-${WHISPER_LANG:-ru}}"
[[ -f $IN ]] || { echo "stt: no such file: $IN" >&2; exit 1; }
[[ -f $MODEL ]] || { echo "stt: model missing: $MODEL" >&2; exit 1; }

# whisper.cpp only accepts 16 kHz mono PCM; Telegram voice arrives as OGG/Opus.
WAV=$(mktemp --suffix=.wav)
trap 'rm -f "$WAV"' EXIT
ffmpeg -nostdin -loglevel error -y -i "$IN" -ar 16000 -ac 1 -c:a pcm_s16le "$WAV"

"$BIN" -m "$MODEL" -f "$WAV" -l "$LANG_" -nt -np -t 4 2>/dev/null |
  sed 's/^[[:space:]]*//;s/[[:space:]]*$//' | grep -v '^$'
