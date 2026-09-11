---
name: car-control
description: Drive the three-wheeled car chassis (steer/move, camera, ultrasonic, server management). Use whenever a task involves moving the car, taking a snapshot, reading the distance sensor, or the camera/server hangs.
---

# Car (chassis) control

`car.py` talks to the Freenove three-wheeled smart car's own TCP server (port
12345, one connection at a time) and its mjpg-streamer camera feed (port 8090).

## The camera lives on the ARM, not the chassis

Despite the Freenove kit's naming, `camera_pan`/`camera_tilt`/`camera_center` (the
`>Camera Left/Right/Up/Down` protocol commands) actually drive the **ultrasonic
sensor's** pan/tilt mount, confirmed live (`look_around()` produced near-identical
frames at every pan angle - because the real camera, mounted on the arm's wrist,
never moved). To actually look around, move the ARM (see arm-control skill)
and/or drive the chassis - not `car.py`'s pan/tilt.

## Server/camera health - check this first

`car.py status` reports the command port, camera port, and whether `Main.py` is
running. Two independent recovery commands:
- `car.py restart-camera` - kills and relaunches JUST mjpg-streamer. Use this for
  the **very common** case where the command port still answers but the camera
  port times out or refuses connections (mjpg-streamer hangs on its own,
  especially after the USB camera is bumped/re-enumerates). This happened several
  times per session - if a snapshot call times out, restart the camera and retry
  before assuming anything else is wrong.
- `car.py restart-server` - full restart of `Main.py` (needed if the command port
  itself is down, e.g. after a reboot; runs headless via `QT_QPA_PLATFORM=offscreen`
  so no X server is needed).

**`Main.py` runs under systemd as `car-server.service`** (installed 2026-08-29;
before that it was started by hand and did NOT survive a reboot, so the
net-watchdog's automatic reboot could leave the chassis and camera silently dead).
It is `enabled`, so a reboot brings it back on its own, and `Restart=always`
covers a crash. `car.py restart-server` detects the unit and goes through
`systemctl restart`; do the same by hand rather than killing `Main.py`, because
systemd will immediately restart what you killed and any hand-launched copy then
fights it for port 12345.

Never `pkill -f Main.py` from a shell whose own command line contains that
string - the pattern matches the shell itself and kills the session mid-script.
Use `systemctl restart car-server.service`.

`find_camera_device()` probes `/dev/video*` for whichever one is the real USB
camera (it does NOT reliably stay `/dev/video0` - unplugging/replugging or a USB
bus reset can shift it to video1, video2, etc, and several on-SoC codec/ISP video
nodes also advertise MJPG/YUYV formats and will false-match on format alone). It
checks each candidate's own driver name via `v4l2-ctl -d <dev> -D` (looking for
`uvcvideo`) rather than the aggregate `--list-devices` output, which prints a
human-readable card name, not the driver name.

## Ultrasonic - flaky, read with a median filter

The distance sensor drops out often (a single read returns `0.0`, especially
pointed straight ahead at some angles) - this is a known hardware/wiring quirk, not
a bug to chase down. Use `car._read_sonic_median(samples, timeout)` (median of a
few reads, discarding zeros) rather than a single `car.ultrasonic()` call whenever
the reading matters. `car.radar_sweep()` does a full back-and-forth scan with this
filtering built in. A flat/thin object lying on the floor may simply not reflect
the ultrasonic beam at all - don't expect it to "see" every obstacle a camera would.

### `depth.py` - the Aurora930 depth map, a much better near-field range sense

Since 2026-09-09 the wrist camera is an Aurora930 RGB-D and `aurora-camera.service`
serves its depth map at `:8090/depth`. `depth.py` reads it:

- `depth.py` - readout + ASCII near/far map
- `depth.py ranges [--json]` - left / centre / right / nearest distances (metres)
- `depth.py clear [MM]` - exit 0 if >= MM clear ahead (default 400), else 1

It is a 640x400 metric field, not one beam, so it catches things the ultrasonic
misses and gives per-direction distances in one shot. Two limits, both measured:
it is **strong at `deck`/`floor` arm pitch** (textured near floor + furniture, ~30%
coverage, sofa at 0.63 m read true) and **weak at `horizon`** (plain wall + the
backlit kitchen counter drop coverage to ~9%). Structured light also returns 0
closer than ~15 cm and past ~4 m. So: use `depth.py` for the near field with the
arm looking down, keep the ultrasonic for the horizontal far field, and treat a
low-coverage reading as "don't know", not "clear". The camera is on the arm, so
aim first (`nav.py lookout --view deck`) then read.

#### Low coverage on an OBJECT usually means it is too close, not too dark

The costliest hour of 2026-09-11 went here. Coverage on a yellow bag ran 14-30%,
and I called it dark fabric in shadow. It was not: **85.5% of the bag was nearer
than the sensor could measure**, and silence was the only thing it could return.

Silence has two causes - a surface that does not reflect, and a surface too close
to triangulate - and they look identical in one number. The *pattern* separates
them. Slice the object's pixels by image row and print coverage per band:

    rows 40-119   coverage 0.0%     <- nearest rows, all silent
    rows 120-359  coverage 6-26%    <- answers, ~240 mm
    => the near part is inside the blind zone; you are already on top of it

If the NEAR rows are empty and the FAR rows answer, you have arrived. If coverage
is uniformly poor across near and far alike, then it is the surface.

#### Range an object by its 5th percentile, never its median

Same run, same object: I took the median depth over the bag (541 mm) as "the
distance to the bag". A big object spans a big depth range, and here only its far
side had answered at all - so the median described the FAR rim while the near rim
was already 207 mm away. I concluded the bag was 54 cm off and planned another
35 cm of driving while the gripper was nearly over it.

You care about the near face - the thing you will hit, or reach for. Use `p05` or
the minimum over the object's pixels. `sectors()` already does this per third;
do the same when you mask out a single object.

#### The frame's minimum depth is NOT the sensor's minimum range

Across one evening I quoted "the sensor's minimum range" as ~700 mm, then ~490,
then ~388, then 207 mm. Every one of those was just the nearest object in view.
It is never a property of the instrument, and reasoning from it (as I did: "the
bag returns nothing because it is inside the 70 cm blind zone") builds a whole
plan on a number that changes when someone walks past.

## Steering and movement - no odometry, weak steering

- `car.steer(direction, angle)` sets front-wheel angle (10-60°, "center" = 90).
  `car.move(direction, speed, seconds)` / `car.drive_mm(direction, mm)` (open-loop,
  calibrated from `CAL_SPEED`/`CAL_SECONDS`/`CAL_MM` - not odometry, don't trust the
  requested distance as the actual one).
- The car's steering is weak - a single steer+drive pulse turns the body only a
  little. To rotate roughly in place (minimal net translation), do a K-turn:
  forward+steer one way, then backward+steer the OTHER way (both phases rotate the
  body the same rotational sense while the net translation ~cancels). See
  `kturn.py` for the pattern.
- **Budget the turn before you plan a route.** Measured 2026-08-29: one
  `nav.py turn left 50 55 1.0` produced **+2.4 deg**; five more at full steer
  (angle 60, speed 60, 1.2 s) claimed +25 deg in total, and most were flagged low
  confidence. A 90 deg change of heading is therefore *tens* of manoeuvres and a
  real amount of battery. Plan routes as long straight legs and wide arcs, and
  treat "just turn around and look" as expensive, not free. If the robot is
  wedged in a corner, ask the user to move it rather than grinding K-turns.
- **The reported heading is not yet trustworthy.** After five LEFT turns the pose
  read +25 deg CCW, but the view matched what had been scanned at a NEGATIVE
  bearing earlier - a sign or scale error somewhere between `turn`, `NECK_SIGN`
  and the bearing arithmetic that is still unresolved. `nav.py turn` prints its
  own "neither estimate is trustworthy" warning when both ORB and the tile-line
  fallback are weak; believe that warning and confirm with a snapshot.
- For any meaningful driven distance, camera- or ultrasonic-based verification
  beats dead reckoning - both under- and over-shooting by a lot happened this
  session trusting `drive_mm` alone on a longer leg. `car.move_verified()` (sonar
  scan-matching drift check) and `dxyaw.py`/`turncal.py` (camera-based yaw from ORB
  matches) exist for this, though both need enough scene texture/overlap between
  the before/after frames to be reliable - they can fail silently (near-zero
  reported yaw) on a low-texture scene (e.g. camera pointed at a plain curtain).

## Navigate on BEARING, not on distance travelled

The single thing that worked on 2026-09-11's cross-room approach. Bearing to a
target comes from the camera intrinsics alone:

    bearing_deg = degrees(atan((px - cx) / fx))       # fx=418.33, cx=316.55

No camera model, no pitch, no floor plane, no odometry - just the pixel column.
It is the one geometric quantity on this robot I have never caught lying.

So: drive a leg, re-measure the target's bearing, correct, repeat. Over six legs
the bag's bearing went -4.0 -> +9.1 -> +6.3 -> +2.9 -> +8.9 -> +13.3 -> +5.8 ->
in the groove, and the user freed a jammed wheel TWICE during that without my
ever losing the target. Dead reckoning would have been destroyed by either jam.

Two cautions:
- **Convert camera bearing to VEHICLE bearing.** The camera pans with servo 6;
  `BASE_FORWARD = 470` is straight ahead, 4 units/deg. So
  `bearing_veh = bearing_cam - (s6 - 470)/4`. Read s6, do not assume it.
- **Bearing grows as you close in.** A fixed 150 mm lateral offset reads +5.4 deg
  at 1.5 m and +13 deg at 0.7 m. A growing bearing on a straight run is the
  geometry working correctly, not a heading drift - do not "correct" it away.

### An object held in the jaws biases the bearing it occludes

The held bar covered the bag's LEFT edge, so the yellow centroid sat right of
truth and the bearing read +6.3 deg. Panning the camera 5 deg to clear the bar
gave **+2.9 deg** from the same spot. Before believing a centroid, check whether
the blob touches the frame edge or the held object; if it does, pan to unocclude
and re-measure, then subtract the pan.

## Drive distances: measure them, and re-measure after any jam

Rough figures at speed 55, from depth-to-a-fixed-surface used as a ruler:

| pulse | free wheels | one wheel jammed |
|-------|-------------|------------------|
| 0.6 s | 5-11 cm     | 11 cm |
| 1.2 s | **29 cm**   | 21 cm |

Short pulses lose most of their travel to breakaway friction, so they are not a
fraction of the long ones - 0.6 s is far less than half of 1.2 s.

**Do not write a calibration measured during a jam.** This has now bitten twice:
on 2026-09-09 the numbers 1.16/0.44/18.21 cm produced a confident "the drive is
non-linear in duration" that the user retracted ("Колеса были заблокированы"),
and on 2026-09-11 a wheel jammed again mid-approach. When the user says they have
freed a wheel, every distance figure taken before that is suspect - say so and
re-measure rather than quietly keeping it.

## The gate: what it needs, and what to do when it says no

`gated_move.sh {drive|nudge|turn} [--direction ...] -- <car.py args>` is the only
sanctioned way to move. Clearance required: **drive 25 cm, turn 45 cm, nudge 8 cm**,
and with no ultrasonic fitted it demands **twice** that from depth alone.

- **The gate reads the CENTRE sector**, i.e. the direction of travel. It used to
  take the 5th percentile of left+centre+right pooled, which let far surfaces off
  to the sides dilute an obstacle dead ahead: measured 96 cm where the centre read
  70. Fixed 2026-09-11. Consequence for you: **aim the camera along the vehicle
  axis before gating.** A camera panned 5 deg off and pitched 24 deg down feeds
  the gate a reading of the floor and the furniture beside you, not your path.
- **`nudge` is the honest category for positioning against the object you are
  working on.** When the bag was 40 cm away the turn gate correctly refused (45 cm
  needed), and the last few degrees of alignment came from gated *nudges*, which
  is exactly what the category is for. It is not a way to sneak a drive past the
  gate - if you are travelling, it is a drive.
- **With the ultrasonic removed, the gate refuses every turn where anything is
  inside 90 cm** - which, in this room, is almost everywhere. That is the concrete
  answer to "нужен ли ультразвук": yes, not for ranging (it is trusted only under
  30 cm) but because its presence halves the depth margin the gate demands.

## Distance/scale illusions from the wrist camera

The wrist-mounted camera's narrow/close-focus framing made a floor object 1-2m away
look deceptively close and large during a pure vision-based approach one session -
repeated over/under-shoots followed. When judging real-world distance or direction
and the camera's own framing seems ambiguous, an external third-person photo (ask
the user) is far more reliable than guessing from the robot's own camera feed.
