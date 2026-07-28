#!/usr/bin/env python3
"""usb_charger.py -- unplug/replug the real charger's USB block, at THIS car/mount position.

WHY HARDCODED SERVO POSES INSTEAD OF IK: this object is small, dark, and sits on a
cardboard-covered mount with no reliable automatic detector yet (its LED is blue, too
close to pick_eye's bar-blue hue range, and the object is compact rather than elongated,
so the bar detector's shape filters would reject it - see memory
[[real-charger-usb-session-2026-07-28]]). Plain IK-based vertical motion
(pick_eye.goto_vertical) also hit real limits here: pinning the wrist pitch exactly is
only reachable within about +-5deg for a 3cm lift and needed +-12deg for 5cm, so a pure
IK approach silently trades pitch accuracy for reachability in a way that is hard to
predict in advance.

Instead, POSE_GRASP/POSE_EXTRACT/POSE_SEAT below are servo tuples taught LIVE on
2026-07-28 by hand (shoulder(5) and wrist(3) moved together, elbow(4)/base(6) held
fixed) and verified over 3 repeats to pull the block straight up and set it straight
back down without visible drift - which is what keeps it from catching the socket's
edges on the way in/out. If the object width/height at the socket ever changes (a
different plug, a remount), re-teach these poses the same way: from POSE_GRASP, close
the jaws, nudge shoulder+wrist together in small steps while watching a frame each
time, and stop as soon as the object stops sliding sideways in the image.

THIS IS POSITION-SPECIFIC: these are absolute servo values, not a relative motion, so
they are only valid with the car parked in the same spot (and same camera/arm mount)
as when they were taught. Moving the car invalidates all three poses - see the
docstring in rig.py next to the "REAL CHARGER USB" note for the reasoning on why an
autonomous approach needs a vision re-centring step before these poses can be reused.

Run under the venv as root, like everything else that touches the arm:
    sudo /home/astra/tools/venv/bin/python3 usb_charger.py cycle
"""
import subprocess
import sys
import time

ARM = "/home/astra/robotics/arm"
SCRATCH = "/tmp/_usb_charger.jpg"

GRIPPER_OPEN = 515    # clears the block's width, measured live this session
GRIPPER_CLAMP = 820   # commanded well past contact; it stalls around 677-680 on the object

# 6=base, 5=shoulder, 4=elbow, 3=wrist(local tilt) - kin.py's real joint convention,
# see the arm-control skill's note on the arm.py/kin.py naming discrepancy.
POSE_GRASP = {6: 476, 5: 248, 4: 667, 3: 115}     # aligned on the socket, ready to close
POSE_EXTRACT = {6: 476, 5: 268, 4: 667, 3: 103}   # ~3cm pulled straight out, verified x3
POSE_SEAT = {6: 476, 5: 243, 4: 667, 3: 115}      # a touch firmer than POSE_GRASP, for a
                                                   # snug seat before opening on insert

# A stall within this band of GRIPPER_CLAMP means the jaws hit something solid, not just
# air - the same signal used live this session. It does NOT by itself prove the object is
# the USB block (see the wiggle-test caveat in memory [[usb-plug-grasp-attempt]]); it is
# just a cheap sanity check worth logging.
STALL_LO, STALL_HI = 660, 700


def _step(servos, ms=1500):
    moves = ",".join(f"{j}:{v}" for j, v in servos.items())
    subprocess.run([ARM, "step", moves, SCRATCH, str(ms)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)


def _gripper_pos():
    out = subprocess.run(["sudo", "/home/astra/tools/venv/bin/python3",
                          "/home/astra/robotics/arm.py", "status"],
                         capture_output=True, text=True).stdout
    for line in out.splitlines():
        if line.startswith("servo 1"):
            return int(line.split("position=")[1].split()[0])
    return None


def grasp(log=print):
    """Move to POSE_GRASP and close hard. Returns True if the jaws stalled in the
    expected band (a solid contact), False if they closed all the way to CLAMP
    (probably missed - nothing there to stop them)."""
    _step(POSE_GRASP, 1500)
    _step({1: GRIPPER_CLAMP}, 1200)
    p = _gripper_pos()
    ok = p is not None and STALL_LO <= p <= STALL_HI
    log(f"  [grasp] gripper stalled at {p} ({'OK, solid contact' if ok else 'unexpected - check the frame'})")
    return ok


def extract(ms=1500, log=print):
    """From a closed grasp at POSE_GRASP, pull the block straight out to POSE_EXTRACT."""
    _step(POSE_EXTRACT, ms)
    log("  [extract] at POSE_EXTRACT")


def seat_and_release(ms=1500, log=print):
    """Descend to the firm POSE_SEAT and open the jaws, leaving the block seated."""
    _step(POSE_SEAT, ms)
    _step({1: GRIPPER_OPEN}, 1000)
    log("  [seat_and_release] released at POSE_SEAT")


def cycle(pause=1.2, log=print):
    """One full round trip: grasp -> extract -> re-seat -> release. Returns the grasp()
    stall check (the only cheap signal available without a wiggle test)."""
    ok = grasp(log=log)
    time.sleep(pause)
    extract(log=log)
    time.sleep(pause)
    seat_and_release(log=log)
    return ok


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "cycle"
    if cmd == "cycle":
        cycle()
    elif cmd == "grasp":
        grasp()
    elif cmd == "extract":
        extract()
    elif cmd == "seat":
        seat_and_release()
    else:
        print(f"usage: {sys.argv[0]} [cycle|grasp|extract|seat]")


if __name__ == "__main__":
    main()
