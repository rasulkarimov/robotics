// aurora_streamer — pulls aligned RGB + depth from the Deptrum Aurora930 and
// publishes them to a tmpfs directory for aurora_http.py to serve on :8090.
//
// The rest of the robot consumes the camera as an mjpg-streamer HTTP endpoint
// (arm.py CAMERA_URL, car.py CAMERA_PORT). This process owns the SDK; the Python
// shim owns HTTP. Frames are written atomically (tmp + rename) so a reader never
// sees a half-written file.
//
// Output files in --outdir (default /dev/shm/aurora):
//   rgb.jpg      latest colour frame, JPEG
//   depth.raw    latest depth frame, row-major uint16 little-endian, millimetres
//   meta.json    {ts_ms, frame, rgb_w, rgb_h, depth_w, depth_h, center_mm}
//
// Exit non-zero on a persistent device error so systemd restarts it.
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <turbojpeg.h>

#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "deptrum/aurora900_series.h"

using namespace deptrum;
using namespace deptrum::stream;

static std::atomic<bool> g_run{true};
static void on_signal(int) { g_run = false; }

static uint64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

// Atomic publish: write to <path>.tmp in the same dir, then rename over <path>.
static bool write_atomic(const std::string& path, const void* data, size_t len) {
  std::string tmp = path + ".tmp";
  int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) { fprintf(stderr, "open %s: %s\n", tmp.c_str(), strerror(errno)); return false; }
  const char* p = static_cast<const char*>(data);
  size_t off = 0;
  while (off < len) {
    ssize_t n = ::write(fd, p + off, len - off);
    if (n < 0) { ::close(fd); ::unlink(tmp.c_str()); return false; }
    off += n;
  }
  ::close(fd);
  if (::rename(tmp.c_str(), path.c_str()) != 0) { ::unlink(tmp.c_str()); return false; }
  return true;
}

int main(int argc, char** argv) {
  std::string outdir = "/dev/shm/aurora";
  int mode_index = -1;   // -1 => highest-resolution supported mode
  int quality = 80;
  int ir_fps = 15;
  bool log_fps = false;

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() { return (i + 1 < argc) ? argv[++i] : nullptr; };
    if (a == "--outdir") { if (auto v = next()) outdir = v; }
    else if (a == "--mode") { if (auto v = next()) mode_index = atoi(v); }
    else if (a == "--quality") { if (auto v = next()) quality = atoi(v); }
    else if (a == "--ir-fps") { if (auto v = next()) ir_fps = atoi(v); }
    else if (a == "--log-fps") { log_fps = true; }
    else { fprintf(stderr, "unknown arg: %s\n", a.c_str()); return 2; }
  }

  ::mkdir(outdir.c_str(), 0755);
  const std::string rgb_path = outdir + "/rgb.jpg";
  const std::string depth_path = outdir + "/depth.raw";
  const std::string meta_path = outdir + "/meta.json";

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  auto dm = DeviceManager::GetInstance();
  dm->RegisterDeviceConnectedCallback();

  std::vector<DeviceInformation> devs;
  int ret = dm->GetDeviceList(devs);
  if (ret || devs.empty()) { fprintf(stderr, "no Aurora device (ret=%d)\n", ret); return 1; }

  auto dev = dm->CreateDevice(devs[0]);
  if (!dev) { fprintf(stderr, "CreateDevice failed\n"); return 1; }
  fprintf(stderr, "SDK %s\n", dev->GetSdkVersion().c_str());
  if ((ret = dev->Open())) { fprintf(stderr, "Open ret=%d\n", ret); return 1; }
  fprintf(stderr, "device: %s\n", dev->GetDeviceName().c_str());

  std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> modes;
  dev->GetSupportedFrameMode(modes);
  if (modes.empty()) { fprintf(stderr, "no frame modes\n"); return 1; }
  int idx = mode_index >= 0 && mode_index < (int)modes.size()
                ? mode_index
                : (int)modes.size() - 1;  // last = highest res
  auto m = modes[idx];
  fprintf(stderr, "frame mode [%d]: ir=%d rgb=%d depth=%d\n", idx,
          (int)std::get<0>(m), (int)std::get<1>(m), (int)std::get<2>(m));
  if ((ret = dev->SetMode(std::get<0>(m), std::get<1>(m), std::get<2>(m)))) {
    fprintf(stderr, "SetMode ret=%d\n", ret);
    return 1;
  }

  if (auto a900 = std::dynamic_pointer_cast<Aurora900>(dev)) {
    a900->SetIrFps(ir_fps);
    a900->SetLaserCurrent(1450);
    a900->SwitchAlignedMode(true);   // depth aligned to the RGB frame
    a900->DepthCorrection(true);
    a900->EnableUndistortRgb(true);
  }

  // Intrinsics go into meta.json so consumers can turn a pixel shift into an
  // angle without hard-coding a FOV. depth_odom.py needs fx for exactly that.
  Intrinsic ir_intr, rgb_intr;
  Extrinsic extr;
  bool have_intr = (dev->GetCameraParameters(ir_intr, rgb_intr, extr) == 0);
  if (have_intr)
    fprintf(stderr, "rgb intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f (%dx%d)\n",
            rgb_intr.focal_length[0], rgb_intr.focal_length[1],
            rgb_intr.principal_point[0], rgb_intr.principal_point[1],
            rgb_intr.cols, rgb_intr.rows);
  else
    fprintf(stderr, "GetCameraParameters failed; meta.json will omit intrinsics\n");

  Stream* stream = nullptr;
  if ((ret = dev->CreateStream(stream, {kRgbd}))) {
    fprintf(stderr, "CreateStream ret=%d\n", ret);
    return 1;
  }
  if ((ret = stream->Start())) {
    fprintf(stderr, "stream Start ret=%d\n", ret);
    return 1;
  }
  fprintf(stderr, "streaming -> %s\n", outdir.c_str());

  tjhandle tj = tjInitCompress();
  std::vector<uint8_t> jpeg;
  uint64_t frame_no = 0, consec_err = 0;
  auto fps_t0 = std::chrono::steady_clock::now();
  uint64_t fps_n = 0;

  while (g_run) {
    StreamFrames frames;
    ret = stream->GetFrames(frames, 2000);
    if (ret != 0 || frames.count == 0) {
      if (++consec_err >= 20) { fprintf(stderr, "GetFrames failing (ret=%d), exiting\n", ret); break; }
      continue;
    }
    consec_err = 0;

    int rgb_w = 0, rgb_h = 0, depth_w = 0, depth_h = 0;
    uint16_t center_mm = 0;
    bool wrote_rgb = false;

    for (int i = 0; i < frames.count; i++) {
      auto& f = frames.frame_ptr[i];
      if (!f || !f->data) continue;

      if (f->frame_type == kRgbFrame && f->size == f->cols * f->rows * 3) {
        rgb_w = f->cols; rgb_h = f->rows;
        unsigned char* out = nullptr;
        unsigned long out_sz = 0;
        // Frame is packed 24-bit. Deptrum/OpenCV convention is BGR.
        int rc = tjCompress2(tj, static_cast<unsigned char*>(f->data.get()), f->cols, 0, f->rows,
                             TJPF_BGR, &out, &out_sz, TJSAMP_420, quality, TJFLAG_FASTDCT);
        if (rc == 0 && out) {
          write_atomic(rgb_path, out, out_sz);
          wrote_rgb = true;
        }
        if (out) tjFree(out);
      } else if (f->frame_type == kDepthFrame && f->size == f->cols * f->rows * 2) {
        depth_w = f->cols; depth_h = f->rows;
        write_atomic(depth_path, f->data.get(), f->size);
        const uint16_t* d = static_cast<const uint16_t*>(f->data.get());
        center_mm = d[(f->rows / 2) * f->cols + f->cols / 2];
      }
    }

    if (wrote_rgb || depth_w) {
      frame_no++;
      char meta[768];
      int n = snprintf(meta, sizeof(meta),
                       "{\"ts_ms\":%llu,\"frame\":%llu,\"rgb_w\":%d,\"rgb_h\":%d,"
                       "\"depth_w\":%d,\"depth_h\":%d,\"center_mm\":%u",
                       (unsigned long long)now_ms(), (unsigned long long)frame_no,
                       rgb_w, rgb_h, depth_w, depth_h, center_mm);
      if (have_intr && n > 0 && n < (int)sizeof(meta))
        n += snprintf(meta + n, sizeof(meta) - n,
                      ",\"fx\":%.4f,\"fy\":%.4f,\"cx\":%.4f,\"cy\":%.4f,"
                      "\"intr_w\":%d,\"intr_h\":%d",
                      rgb_intr.focal_length[0], rgb_intr.focal_length[1],
                      rgb_intr.principal_point[0], rgb_intr.principal_point[1],
                      rgb_intr.cols, rgb_intr.rows);
      if (n > 0 && n < (int)sizeof(meta))
        n += snprintf(meta + n, sizeof(meta) - n, "}\n");
      write_atomic(meta_path, meta, n);
    }

    if (log_fps && ++fps_n >= 30) {
      auto t1 = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(t1 - fps_t0).count();
      fprintf(stderr, "fps ~%.1f (center %umm)\n", fps_n / dt, center_mm);
      fps_t0 = t1; fps_n = 0;
    }
  }

  fprintf(stderr, "stopping\n");
  stream->Stop();
  dev->DestroyStream(stream);
  dev->Close();
  tjDestroy(tj);
  return (consec_err >= 20) ? 1 : 0;
}
