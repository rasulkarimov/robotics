#!/usr/bin/env python3
"""VLM perception bridge - ask the internal vLLM what is in a camera frame.

Every detector in this repo so far is a hand-tuned colour mask built for one
object (see pick_eye.py's blue/red masks, and the false positives documented in
AGENTS.md). This module is the general alternative: hand a frame and a plain
description ("my socks", "a cardboard box") to the multimodal model behind
MODEL_URL and get pixel coordinates back.

Deliberately PIL-only, no cv2 - so it runs under the SYSTEM python3 (which
car.py uses) as well as the arm venv. Do not add a cv2 import.

Reachability: MODEL_URL lives behind the MotionPro VPN. `probe()` (CLI:
`vision.py probe`) is the one call that says whether vision is available at all;
call it before trusting any other function here.
"""

import argparse
import base64
import io
import json
import os
import re
import sys
import urllib.error
import urllib.request

from PIL import Image, ImageDraw

MODEL_URL = os.environ.get("VISION_URL", "http://10.144.22.51:8000/v1")
MODEL_NAME = os.environ.get("VISION_MODEL", "kimi-k2-5")
TIMEOUT = 120

# The frame is downscaled before it is sent. 512px wide keeps a 640x480 frame
# readable while holding the image cost to a few hundred prompt tokens.
SEND_WIDTH = 512

# Grid overlay. VLMs are poor at naming raw pixel coordinates but good at
# naming a labelled cell, so we draw one and ask for the cell instead.
GRID_COLS = 8
GRID_ROWS = 6
COL_LABELS = "ABCDEFGH"

# What the camera always sees and must never report as an object. The camera is
# on the arm's wrist, so the gripper jaws are permanently in the lower corners -
# the model called them "wheels or feet" until it was told otherwise.
EMBODIMENT = (
    "This is a frame from the camera mounted on the WRIST of a small home robot. "
    "The two dark shapes with orange/red rubber pads in the lower-left and "
    "lower-right corners are the robot's OWN GRIPPER JAWS, always present in "
    "every frame. Never report the jaws, or the robot's own black chassis and "
    "cables, as an object in the room."
)


def _post(messages, max_tokens=1500, json_mode=False):
    """Returns the reply text.

    The model reasons before answering and its reasoning tokens are billed
    against max_tokens, so the budget must be generous or `content` comes back
    empty/truncated with finish_reason='length'. When the answer really did get
    cut off, fall back to the reasoning trace - for a short JSON answer the
    conclusion is usually stated there too. That fallback is OFF under
    `json_mode`: there, an empty content means the budget ran out mid-object, and
    handing the caller the prose trace instead is what produced "unparseable
    reply" in one call out of three even after constrained decoding was on.

    `json_mode` turns on the server's constrained decoding. Asking for JSON in
    the prompt is not enough: the model kept replying with prose ("Let me look
    at this more carefully..."), which `find` could only report as an
    unparseable answer - indistinguishable, to a caller reading exit codes, from
    an honest "not found". With response_format the content comes back as JSON
    and the deliberation lands in the separate `reasoning` field, where it
    belongs. Verified against this endpoint before switching it on.
    """
    payload = {
        "model": MODEL_NAME,
        "max_tokens": max_tokens,
        "messages": messages,
    }
    if json_mode:
        payload["response_format"] = {"type": "json_object"}
    body = json.dumps(payload).encode()
    req = urllib.request.Request(
        MODEL_URL.rstrip("/") + "/chat/completions", body,
        {"Content-Type": "application/json", "Authorization": "Bearer EMPTY"})
    with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
        data = json.loads(r.read().decode())
    choice = data["choices"][0]
    content = (choice["message"].get("content") or "").strip()
    if not content and not json_mode:
        content = (choice["message"].get("reasoning") or "").strip()
    return content


def _encode(img):
    """PIL image -> base64 JPEG, downscaled to SEND_WIDTH."""
    if img.width > SEND_WIDTH:
        h = round(img.height * SEND_WIDTH / img.width)
        img = img.resize((SEND_WIDTH, h), Image.LANCZOS)
    buf = io.BytesIO()
    img.convert("RGB").save(buf, "JPEG", quality=88)
    return base64.b64encode(buf.getvalue()).decode()


def _user(text, img):
    return [{"role": "user", "content": [
        {"type": "text", "text": text},
        {"type": "image_url",
         "image_url": {"url": "data:image/jpeg;base64," + _encode(img)}}]}]


def draw_grid(img):
    """Overlay a labelled GRID_COLS x GRID_ROWS grid. Returns a new image."""
    img = img.convert("RGB").copy()
    d = ImageDraw.Draw(img)
    cw, ch = img.width / GRID_COLS, img.height / GRID_ROWS
    for c in range(1, GRID_COLS):
        d.line([(c * cw, 0), (c * cw, img.height)], fill=(255, 255, 0), width=1)
    for r in range(1, GRID_ROWS):
        d.line([(0, r * ch), (img.width, r * ch)], fill=(255, 255, 0), width=1)
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            label = f"{COL_LABELS[c]}{r + 1}"
            x, y = c * cw + 3, r * ch + 2
            # Black halo so the label survives both the pale floor and the
            # dark chassis.
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                d.text((x + dx, y + dy), label, fill=(0, 0, 0))
            d.text((x, y), label, fill=(255, 255, 0))
    return img


def _cell_to_px(cell, where, w, h):
    """Grid cell label + coarse position inside it -> pixel centre."""
    m = re.fullmatch(r"([A-Ha-h])\s*([1-6])", cell.strip())
    if not m:
        return None
    c = COL_LABELS.index(m.group(1).upper())
    r = int(m.group(2)) - 1
    cw, ch = w / GRID_COLS, h / GRID_ROWS
    fx = {"left": 0.25, "right": 0.75}.get((where or "").lower().split("-")[-1], 0.5)
    fy = {"top": 0.25, "bottom": 0.75}.get((where or "").lower().split("-")[0], 0.5)
    return round((c + fx) * cw), round((r + fy) * ch)


def _json_from(text):
    """Pull the answer object out of a reply that may be fenced or prosy.

    Takes the LAST parseable object carrying a "found" key: when the reply is a
    reasoning trace the model often drafts a candidate answer mid-thought and
    only settles at the end, so the first object is the wrong one to trust.
    """
    dec = json.JSONDecoder()
    best = None
    for i, ch in enumerate(text):
        if ch != "{":
            continue
        try:
            obj, _ = dec.raw_decode(text[i:])
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict) and "found" in obj:
            best = obj
    return best


def probe():
    """Is the vision model reachable? Returns (ok, detail)."""
    try:
        req = urllib.request.Request(MODEL_URL.rstrip("/") + "/models",
                                     headers={"Authorization": "Bearer EMPTY"})
        with urllib.request.urlopen(req, timeout=10) as r:
            ids = [m["id"] for m in json.loads(r.read().decode())["data"]]
        if MODEL_NAME not in ids:
            return False, f"model {MODEL_NAME!r} not served; available: {ids}"
        return True, f"{MODEL_NAME} at {MODEL_URL}"
    except (urllib.error.URLError, OSError, KeyError, ValueError) as e:
        return False, f"{type(e).__name__}: {e} (VPN down? see motionpro.service)"


def describe(path):
    """Free-form description of the scene."""
    img = Image.open(path)
    return _post(_user(
        EMBODIMENT + "\n\nDescribe what you see: the objects in the room, the "
        "floor surface, and roughly how far away the nearest ones are. Be "
        "concrete and brief.", img))


def ask(path, question):
    """Answer an arbitrary question about the frame."""
    img = Image.open(path)
    return _post(_user(EMBODIMENT + "\n\n" + question, img))


def find(path, target, save_grid=None):
    """Locate `target` in the frame.

    Returns a dict: {"found": bool, "cx": int, "cy": int, "cell": str,
    "confidence": "high|medium|low", "why": str}. Coordinates are in the
    ORIGINAL image's pixel space.

    `found: False` is a real answer, not an error - the caller is expected to
    turn or drive and look again rather than treat a miss as a failure.
    """
    img = Image.open(path)
    w, h = img.size
    grid = draw_grid(img)
    if save_grid:
        grid.save(save_grid)

    prompt = (
        EMBODIMENT + "\n\n"
        f"A yellow {GRID_COLS}x{GRID_ROWS} grid is drawn over the frame. Columns "
        f"are lettered {COL_LABELS[0]}-{COL_LABELS[GRID_COLS - 1]} left to right, "
        f"rows are numbered 1-{GRID_ROWS} top to bottom, so each cell has a label "
        "like C4.\n\n"
        f"TASK: find {target}.\n\n"
        "Answer with ONLY a JSON object, no other text:\n"
        '{"found": true/false, "cell": "<label of the cell containing the CENTRE '
        'of the object, e.g. C4>", "where": "<one of: top-left, top, top-right, '
        'left, center, right, bottom-left, bottom, bottom-right - where inside '
        'that cell>", "confidence": "high|medium|low", "why": "<one short '
        'sentence: what you actually see that makes you say this>"}\n\n'
        "If the object is not in this frame, answer found=false and say in "
        "'why' what IS on the floor instead. Do not guess a location to be "
        "helpful - a false sighting is worse than a miss."
    )
    # Localising costs far more reasoning than describing: the model walks the
    # grid cell by cell. 1500 tokens truncated a positive answer in testing.
    raw = _post(_user(prompt, grid), max_tokens=3500, json_mode=True)
    obj = _json_from(raw)
    if obj is None:
        return {"found": False, "cx": None, "cy": None, "cell": None,
                "confidence": "low", "why": f"unparseable reply: {raw[:200]}"}

    out = {"found": bool(obj.get("found")),
           "cell": obj.get("cell"),
           "confidence": obj.get("confidence", "low"),
           "why": obj.get("why", ""),
           "cx": None, "cy": None}
    if out["found"] and out["cell"]:
        px = _cell_to_px(out["cell"], obj.get("where"), w, h)
        if px:
            out["cx"], out["cy"] = px
        else:
            out["found"] = False
            out["why"] = f"bad cell label {out['cell']!r}; " + out["why"]
    return out


def cmd_probe(_args):
    ok, detail = probe()
    print(("OK: " if ok else "UNAVAILABLE: ") + detail)
    return 0 if ok else 1


def cmd_describe(args):
    print(describe(args.image).strip())
    return 0


def cmd_ask(args):
    print(ask(args.image, args.question).strip())
    return 0


def cmd_find(args):
    r = find(args.image, args.target, save_grid=args.save_grid)
    print(json.dumps(r, ensure_ascii=False))
    return 0 if r["found"] else 2


def cmd_grid(args):
    draw_grid(Image.open(args.image)).save(args.out)
    print(f"saved {args.out}")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("probe", help="is the vision model reachable?")

    d = sub.add_parser("describe", help="free-form description of a frame")
    d.add_argument("image")

    a = sub.add_parser("ask", help="ask an arbitrary question about a frame")
    a.add_argument("image")
    a.add_argument("question")

    f = sub.add_parser("find", help="locate an object; prints JSON, exit 2 if absent")
    f.add_argument("image")
    f.add_argument("target", help='plain description, e.g. "a pair of socks"')
    f.add_argument("--save-grid", help="also write the grid-overlaid frame here")

    g = sub.add_parser("grid", help="write the grid overlay (for eyeballing it)")
    g.add_argument("image")
    g.add_argument("out")

    args = p.parse_args()
    return {"probe": cmd_probe, "describe": cmd_describe, "ask": cmd_ask,
            "find": cmd_find, "grid": cmd_grid}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
