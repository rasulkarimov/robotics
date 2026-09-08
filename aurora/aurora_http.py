#!/usr/bin/env python3
"""aurora_http — serves the Aurora930 frames that aurora_streamer publishes to
tmpfs, on the mjpg-streamer-compatible port 8090 so the rest of the robot
(arm.py CAMERA_URL, car.py CAMERA_PORT, camera_watch.py) keeps working unchanged.

Endpoints:
  GET /?action=snapshot   latest RGB frame, image/jpeg          (the one everything uses)
  GET /?action=stream     multipart/x-mixed-replace MJPEG stream
  GET /depth              latest depth, raw row-major uint16 LE, millimetres
                          headers: X-Width X-Height X-Unit(mm) X-Format(uint16-le)
  GET /depth.png          latest depth as a 16-bit grayscale PNG (preview/tools)
  GET /meta               meta.json from the streamer
  GET / or /health        plain-text status

503 is returned when the newest frame is older than --stale seconds (default 5):
a stale JPEG served as current is how a frozen camera went unnoticed before.
"""
import argparse
import io
import json
import os
import struct
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

OUTDIR = "/dev/shm/aurora"
STALE_S = 5.0
STREAM_FPS = 10


def _age(path):
    try:
        return time.time() - os.stat(path).st_mtime
    except OSError:
        return None


def _read(path):
    with open(path, "rb") as f:
        return f.read()


def _meta():
    try:
        return json.loads(_read(os.path.join(OUTDIR, "meta.json")))
    except (OSError, ValueError):
        return {}


def _depth_dims():
    m = _meta()
    w, h = m.get("depth_w"), m.get("depth_h")
    if w and h:
        return int(w), int(h)
    # fall back to size assuming the streamer's default 640x400
    n = os.path.getsize(os.path.join(OUTDIR, "depth.raw")) // 2
    return (640, n // 400) if n % 400 == 0 else (n, 1)


def _png16_gray(data16le, w, h):
    """Minimal 16-bit grayscale PNG encoder (no numpy/PIL dependency)."""
    def chunk(tag, payload):
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 16, 0, 0, 0, 0)  # 16-bit, grayscale
    # PNG samples are big-endian; the streamer writes little-endian. Byteswap each
    # row and prepend a 0 (no-op) filter byte.
    row_le = w * 2
    mv = memoryview(data16le)
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        row = bytearray(mv[y * row_le:(y + 1) * row_le])
        row[0::2], row[1::2] = row[1::2], row[0::2]
        raw.extend(row)
    return sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(bytes(raw), 6)) + chunk(b"IEND", b"")


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _fresh(self, name):
        path = os.path.join(OUTDIR, name)
        age = _age(path)
        if age is None:
            self._send(503, "text/plain", b"no frame yet\n")
            return None
        if age > STALE_S:
            self._send(503, "text/plain", f"stale frame ({age:.1f}s old)\n".encode())
            return None
        return path

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        action = q.get("action", [None])[0]

        if (u.path == "/" and action is None) or u.path == "/health":
            m = _meta()
            age = _age(os.path.join(OUTDIR, "rgb.jpg"))
            ok = age is not None and age <= STALE_S
            body = f"aurora_http {'OK' if ok else 'STALE'} age={age} meta={m}\n".encode()
            self._send(200 if ok else 503, "text/plain", body)
            return

        if action == "snapshot" or (u.path == "/snapshot"):
            path = self._fresh("rgb.jpg")
            if path:
                self._send(200, "image/jpeg", _read(path),
                           {"Cache-Control": "no-store"})
            return

        if action == "stream":
            self._stream()
            return

        if u.path == "/depth":
            path = self._fresh("depth.raw")
            if path:
                w, h = _depth_dims()
                self._send(200, "application/octet-stream", _read(path),
                           {"X-Width": str(w), "X-Height": str(h),
                            "X-Unit": "mm", "X-Format": "uint16-le",
                            "Cache-Control": "no-store"})
            return

        if u.path == "/depth.png":
            path = self._fresh("depth.raw")
            if path:
                w, h = _depth_dims()
                self._send(200, "image/png", _png16_gray(_read(path), w, h),
                           {"Cache-Control": "no-store"})
            return

        if u.path == "/meta":
            self._send(200, "application/json",
                       _read(os.path.join(OUTDIR, "meta.json")))
            return

        self._send(404, "text/plain", b"not found\n")

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while True:
                path = os.path.join(OUTDIR, "rgb.jpg")
                if _age(path) is not None:
                    data = _read(path)
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                time.sleep(1.0 / STREAM_FPS)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    global OUTDIR, STALE_S
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", default=OUTDIR)
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--stale", type=float, default=STALE_S)
    args = ap.parse_args()
    OUTDIR = args.outdir
    STALE_S = args.stale
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"aurora_http on :{args.port} serving {OUTDIR} (stale>{STALE_S}s -> 503)", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
