// One-shot capability probe for the Aurora930: device name, supported frame
// modes, supported stream types, and one frame per stream type with its real
// format/dims. Throwaway diagnostic — the streamer is built from what this prints.
#include <cstdio>
#include <memory>
#include <vector>
#include <tuple>
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "deptrum/aurora900_series.h"

using namespace deptrum;
using namespace deptrum::stream;

static const char* fmt_name(ImageFormat f) {
  switch (f) {
    case kRaw8: return "Raw8"; case kRaw10: return "Raw10"; case kRaw12: return "Raw12";
    case kRaw16: return "Raw16"; case kRgb888: return "Rgb888"; case kRgba: return "Rgba";
    case kYuv420Nv12: return "Nv12"; case kYuv420Nv21: return "Nv21"; case kJpeg: return "Jpeg";
    default: return "Invalid";
  }
}
static const char* ft_name(FrameType t) {
  switch (t) {
    case kRgbFrame: return "Rgb"; case kIrFrame: return "Ir"; case kDepthFrame: return "Depth";
    case kPointCloudFrame: return "PointCloud"; case kRgbdPointCloudFrame: return "RgbdPC";
    case kSemanticFrame: return "Semantic"; default: return "?";
  }
}

int main() {
  auto dm = DeviceManager::GetInstance();
  dm->RegisterDeviceConnectedCallback();
  std::vector<DeviceInformation> devs;
  int ret = dm->GetDeviceList(devs);
  if (ret || devs.empty()) { printf("no device (ret=%d)\n", ret); return 1; }

  auto dev = dm->CreateDevice(devs[0]);
  printf("SDK version: %s\n", dev->GetSdkVersion().c_str());
  ret = dev->Open();
  if (ret) { printf("Open failed ret=%d\n", ret); return 1; }
  printf("device name: %s\n", dev->GetDeviceName().c_str());

  std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> modes;
  dev->GetSupportedFrameMode(modes);
  printf("\nsupported (ir,rgb,depth) frame modes: %zu\n", modes.size());
  for (size_t i = 0; i < modes.size(); i++)
    printf("  [%zu] ir=%d rgb=%d depth=%d\n", i,
           (int)std::get<0>(modes[i]), (int)std::get<1>(modes[i]), (int)std::get<2>(modes[i]));

  std::vector<StreamType> sts;
  dev->GetSupportedStreamType(sts);
  printf("\nsupported stream types: ");
  for (auto s : sts) printf("%d ", (int)s);
  printf("\n");

  // Apply the first mode, aurora-specific setup, then sample each single-component stream.
  auto m = modes.empty() ? std::make_tuple(kInvalid, kInvalid, kInvalid) : modes[0];
  dev->SetMode(std::get<0>(m), std::get<1>(m), std::get<2>(m));
  auto a900 = std::dynamic_pointer_cast<Aurora900>(dev);
  if (a900) { a900->SetIrFps(15); a900->SetLaserCurrent(1450); a900->SwitchAlignedMode(true);
              a900->DepthCorrection(true); a900->EnableUndistortRgb(true); }

  for (StreamType st : std::vector<StreamType>{kRgb, kDepth, kRgbd, kRgbdIr}) {
    Stream* stream = nullptr;
    ret = dev->CreateStream(stream, {st});
    if (ret) { printf("\nstreamtype %d: CreateStream ret=%d\n", (int)st, ret); continue; }
    if ((ret = stream->Start())) { printf("\nstreamtype %d: Start ret=%d\n", (int)st, ret);
                                   dev->DestroyStream(stream); continue; }
    StreamFrames frames;
    ret = stream->GetFrames(frames, 3000);
    printf("\nstreamtype %d: GetFrames ret=%d count=%d\n", (int)st, ret, frames.count);
    for (int i = 0; i < frames.count; i++) {
      auto& f = frames.frame_ptr[i];
      printf("   frame[%d] type=%s fmt=%s %dx%d bpp=%d size=%d\n", i,
             ft_name(f->frame_type), fmt_name(f->image_format),
             f->cols, f->rows, f->bits_per_pixel, f->size);
    }
    stream->Stop();
    dev->DestroyStream(stream);
  }
  dev->Close();
  return 0;
}
