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
- For any meaningful driven distance, camera- or ultrasonic-based verification
  beats dead reckoning - both under- and over-shooting by a lot happened this
  session trusting `drive_mm` alone on a longer leg. `car.move_verified()` (sonar
  scan-matching drift check) and `dxyaw.py`/`turncal.py` (camera-based yaw from ORB
  matches) exist for this, though both need enough scene texture/overlap between
  the before/after frames to be reliable - they can fail silently (near-zero
  reported yaw) on a low-texture scene (e.g. camera pointed at a plain curtain).

## Distance/scale illusions from the wrist camera

The wrist-mounted camera's narrow/close-focus framing made a floor object 1-2m away
look deceptively close and large during a pure vision-based approach one session -
repeated over/under-shoots followed. When judging real-world distance or direction
and the camera's own framing seems ambiguous, an external third-person photo (ask
the user) is far more reliable than guessing from the robot's own camera feed.
