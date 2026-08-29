---
name: vision
description: Ask what is in a camera frame - find an arbitrary object by plain description, describe a scene, or verify a grasp. Use whenever a task needs to SEE something there is no colour-mask detector for, and before writing a new hand-tuned detector.
---

# Seeing with the VLM

`vision.py` sends a camera frame to the internal multimodal model and asks about
it in plain language. Verified working 2026-08-29.

Before this, every object needed its own hand-tuned colour mask (`pick_eye.see()`
and friends), and those masks kept locking onto the wrong thing - the power
strip's glow read as "the object", and the jaw-marker detector broke completely
on a cardboard background. Reach for `vision.py` first; write a mask only when
you need a tight servo loop at 10 Hz, which the model cannot do.

## Commands

    python3 vision.py probe                       # is vision available at all?
    python3 vision.py describe FRAME.jpg          # free-form scene description
    python3 vision.py ask FRAME.jpg "QUESTION"    # arbitrary question
    python3 vision.py find FRAME.jpg "a pair of socks"   # JSON, exit 2 if absent
    python3 vision.py grid FRAME.jpg OUT.jpg      # see the grid the model sees

Plain `python3` is correct - the module is deliberately PIL-only with no cv2, so
it runs under the same interpreter as `car.py`. Do not add a cv2 import.

`find` prints `{"found":..., "cx":..., "cy":..., "cell":..., "confidence":...,
"why":...}` with coordinates in the original frame's pixel space.

## Check `probe` first

The model lives on the internal endpoint behind the MotionPro VPN. If the VPN is
down, every call fails. `probe` is the one cheap call that tells you whether
vision is available; run it before building a plan that depends on seeing.

## What it is good and bad at

- **Good:** naming objects, saying what a scene is, answering "is X present",
  and - importantly - saying **no**. On two frames with no socks it answered
  found=false and correctly named the black bag and the blue rug that were
  actually there. It does not invent sightings to be helpful.
- **Coarse, not precise:** localisation comes from a labelled 8x6 grid drawn over
  the frame, because the model names a grid cell far more reliably than a pixel.
  A test plug came back in cell `D2`, about 25 px off its true centre. That is
  good enough to AIM, then hand over to the existing Jacobian servo loop in
  `pick_eye.py` for the last centimetres. Never feed `cx,cy` straight into a
  grasp.
- **Slow:** about **60 s per call**. A five-bearing sweep costs five minutes of
  motor idling and battery. Vision is a deliberate act; never poll it in a loop.
  Look at frames yourself first if you can, and spend model calls on the ones
  that matter.

## Searching for an object in the room

Finding something is a search problem, not a detection problem - this is what
actually failed on the first errand. Two rules learned the hard way:

1. **Pick the right lookout pitch** (`nav.py`, `LOOKOUT_PITCH`). `--view floor`
   (servo5=735) shows the floor from ~1 to 3 m and is what you want for an object
   lying on the ground. The old default `deck` (682) shows about a metre of bare
   tile - ten bearings were swept that way and the room was never in frame.
   `--view horizon` (780) shows furniture and doorways, for working out where the
   robot is.
2. **The arm cannot look behind.** Base rotation reaches about +/-105 deg, so a
   whole rear sector is invisible and the chassis has to turn - which it does very
   slowly (see the car-control skill). Before concluding an object is not in the
   room, check whether it was simply behind the robot.

## Verifying a grasp

The gripper's own reading cannot tell empty from held - 686 vs 678 servo units is
not a reliable difference. After closing the jaws, snapshot and ask:

    python3 vision.py ask FRAME.jpg "Are the gripper jaws holding an object? Answer yes or no and say what."

Use this together with the wiggle test, not instead of it.

## Two traps in the model itself

- **The token budget fails silently.** It reasons before answering and the
  reasoning is billed against `max_tokens`. Too small a budget returns an EMPTY
  answer, which looks exactly like "not found". `find` already asks for 3500;
  if you write a new call, give it room and fall back to the `reasoning` field
  when `content` is empty.
- **It does not know its own body.** It called the gripper jaws "wheels or feet"
  until told otherwise. The `EMBODIMENT` preamble in `vision.py` says the pads in
  the lower corners are the robot's own jaws; keep it in any new prompt, or the
  model will report the robot's own hardware as an object in the room.
