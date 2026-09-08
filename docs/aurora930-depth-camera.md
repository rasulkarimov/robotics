# Aurora930 depth camera — what it is and how it was made to work

Installed 2026-09-08 on the Pi 4 (Debian 13 trixie, aarch64, glibc 2.41).

## It is not a webcam, and `uvcvideo` will never bind it

The "Aurora930 USB camera" is a **Deptrum Aurora 930**: a 3D structured-light
RGB-D camera (IR dot projector + IR camera + RGB camera). 640×400 / 480×300 /
320×200, 5–15 fps, working range 0.3–3 m, ~8 mm depth accuracy at 1 m.

On the bus it is `3251:1930`, and its single interface is **vendor-specific**
(`bInterfaceClass 0xFF`, two bulk endpoints, `0x81` IN / `0x02` OUT). The
configuration string says "Video", which is misleading — there are no UVC
descriptors. Forcing the class driver onto it proves the point:

```
uvcvideo 1-1.4:1.0: Found UVC 0.00 device Aurora 930 (3251:1930)
uvcvideo 1-1.4:1.0: No valid video chain found.
```

So: **no `/dev/video*` node ever appears for it.** Every part of the existing
camera path — `mjpg-streamer` with `input_uvc.so`, port 8090, `car.py`'s
`find_camera_device()` (which explicitly probes for a `uvcvideo` driver name) —
cannot use this camera. That is not a bug to fix; it is a different class of
device. Don't go looking for a kernel module: there isn't one, and there won't
be. The "driver" is a userspace SDK talking libusb.

## Getting the aarch64 SDK

Hiwonder's docs only publish `x86_64` builds, and every download is a Google
Drive *folder* URL, which `curl` cannot walk. `gdown` can:

```bash
pip3 install --break-system-packages gdown
gdown --folder 'https://drive.google.com/drive/folders/1rrcmmeSR_RMuhvtFnmy8zZQXzp5e3sA0'
```

The aarch64 builds do exist, they are just not linked from the docs page. Direct
file ids, in case the folders move:

| file | Drive id | contains |
| --- | --- | --- |
| `deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz` | `1kNR6g-jtqmIQ9ehfYbPZwkOR-AHv58Of` | **SDK v1.1.19, lib + headers matched** |
| `deptrum-ros-driver-aurora930-aarch64-0.7.4-source.tar.gz` | `1JdA2dRfzyL0c-tFhl8aAlLgVX7z7tom1` | SDK v1.1.4 (older) |
| `deptrum-scope-linux-aarch64-gcc-v3.6.17.tar.gz` | `16g8HNfv3vbR7KQlx43pYIMMpD3uW2srD` | Qt GUI viewer |

The useful part of the ROS driver tarball is **not** the ROS driver — it is
`ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/`, which is the plain
C++ SDK: `lib/`, `include/`, `docs/`, `scripts/`, and `samples/bin/` with
**prebuilt aarch64 binaries** (`hello_deptrum`, `sample_lite`, `sample`,
`firmware_upgrade`). Take that directory and ignore the rest.

Version matters: pair the `.so` with the headers it shipped with. Mixing a lib
from the GUI bundle with headers from a driver bundle is how you end up chasing
phantom ABI problems.

## What is installed

- `/etc/udev/rules.d/99-deptrum-libusb.rules` — one line, `idVendor=="3251"`,
  `MODE:="0666"`. Without it only root can open the device.
- `/usr/local/lib/libdeptrum_stream_aurora900.so.1.1.19` + symlink,
  `/etc/ld.so.conf.d/deptrum.conf`, `ldconfig`.
- `/home/astra/tools/deptrum-aurora930/sdk/` — the whole SDK tree.
- `/home/astra/tools/deptrum-aurora930/archives/` — the source tarballs.

No ROS. This is Debian 13, not Ubuntu 22.04; ROS2 Humble has no binaries here and
building it from source would rewrite half the system for no gain, since nothing
in this repo speaks ROS. The standalone SDK is the whole story.

## ABI: a non-issue, and why

The SDK ships built for Ubuntu 18.04. `objdump -T` shows the highest symbol
version it needs is **`GLIBC_2.17`**; this box has 2.41. glibc is backwards
compatible, so the prebuilt binaries just run. Confirmed two ways: the shipped
`hello_deptrum` runs as-is, and `hello_deptrum.cc` compiles natively against the
v1.1.19 headers with the system toolchain:

```bash
g++ -std=c++14 -O2 -I include samples/src/sample_hello_deptrum/hello_deptrum.cc \
    -L lib -ldeptrum_stream_aurora900 -lpthread -o hello_deptrum_native
```

(g++ 14.2. `cmake` is not installed; the full `samples/` build also wants
`libopencv-dev`, which was not worth pulling in when the prebuilt binaries work.)

## Verified working

`samples/bin/sample_lite`, 640×400, align + depth-correction on, all four stream
shapes at a steady **~14.7 fps** against a 15 fps target:

| stream | fps |
| --- | --- |
| RGB + Depth + PointCloud | 14.7 |
| RGB + Depth + IR | 14.7 |
| Depth | 14.7 |
| PointCloud | 14.7 |

Captured frames confirm real data, not just a running pipeline: depth is 16-bit
millimetres (a scene measured 178–3624 mm, ~43–78 % valid pixels depending on
surfaces), IR is clean 8-bit, and the RGB-D point cloud carries per-point colour.

The sample's menu differs between SDK versions — v1.1.19 asks for the **device
index first** (`0`), then frame mode, IR fps, align, depth correction, stream
types, loop count. Feeding v1.1.4's answer sequence to v1.1.19 silently opens
garbage (`ir_pid:0x55`, "Creata Device failed!"). Drive it with, per line:
`0`, `2` (640×400), `15`, `1`, `1`, stream type, `0`, `-1`, then `q` to stop.

## USB, and one thing not to chase

The camera sits on the Pi's USB 2.0 hub at 480 Mbps. That is correct and not
worth "fixing": the device is `bcdUSB 2.00` — it cannot negotiate USB 3, and
640×400 at 15 fps fits in 480 Mbps with room to spare. `dmesg` showed no resets,
no bandwidth complaints, and no disconnects across all the streaming tests.

## Still open

Nothing consumes these streams yet. The SDK is C++ only — there is no Python
binding — so wiring the camera into this robot means writing a bridge: grab RGB
from the SDK and republish it as MJPEG on a port, so `vision.py`, `safety.py`
and everything else that expects the old feed keeps working, with depth exposed
separately for whatever wants it. That is a real piece of work, not a config
change.

Note also that if the old UVC camera has been removed, the `:8090` path
currently has no source at all, and `camera_watch.py` is watching for a camera
that cannot come back.
