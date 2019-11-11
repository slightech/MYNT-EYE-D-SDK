#include "colorizer_p.h"

#include "types_internal.h"

MYNTEYE_BEGIN_NAMESPACE

void ColorizerPrivate::Init(float z14_far,
    bool is_8bits,
    std::shared_ptr<CameraCalibration> calib_params) {
  is_8bits_ = is_8bits;
  calib_params_ = calib_params;
  ComputeZDTable(calib_params_);
}

void ColorizerPrivate::CachedDepthBuffer(const Image::pointer& depth_buf) {
  depth_buf_cached_ = depth_buf;
}

Image::pointer ColorizerPrivate::Process(
    const Image::pointer& depth_raw,
    const ImageFormat& out_format) {
  if (depth_raw->format() != ImageFormat::DEPTH_RAW) {
    // ignored if input is not depth raw
    return depth_raw;
  }

  // 8bits, usb2: depth_buf != depth_raw, use cached depth buffer
  // 14bits, usb3: depth_buf == depth_raw
  auto depth_buf = is_8bits_ ? depth_buf_cached_ : depth_raw;

  if (out_format == ImageFormat::DEPTH_RAW) {
    return depth_raw;
  } else if (out_format == ImageFormat::DEPTH_GRAY) {
    // by image to
    return depth_raw->To(ImageFormat::DEPTH_GRAY);
  } else if (out_format == ImageFormat::DEPTH_GRAY_24) {
    // by color palette
    return Process(depth_buf, DepthMode::DEPTH_GRAY);
  } else if (out_format == ImageFormat::DEPTH_BGR
      || out_format == ImageFormat::DEPTH_RGB) {
    // by color palette
    auto &&depth_rgb = Process(depth_buf, DepthMode::DEPTH_COLORFUL);
    if (out_format == ImageFormat::DEPTH_BGR) {
      return depth_rgb->To(ImageFormat::DEPTH_BGR);
    } else {
      return depth_rgb;
    }
  } else {
    // ignored if output is unaccepted
    return depth_raw;
  }
}

void ColorizerPrivate::ComputeZDTable(
    std::shared_ptr<CameraCalibration> info) {
  float fx = info->CamMat1[0];
  float baseline = -info->TranMat[0];
  float res = fx * baseline;
  // std::cout << "fx:" << fx << "  baseline: " << baseline << std::endl;
  ZD_table_[0] = 0;
  for (int i = 1; i < 256; i++) {
    ZD_table_[i] = static_cast<std::uint16_t>(res / i);
  }
}

void ColorizerPrivate::AdaptU2Raw(unsigned char *src, unsigned char *dst,
    int width, int height) {
  unsigned short * depth = (unsigned short *)dst;   // NOLINT
  for (int i = 0; i < width * height; i += 2) {
    depth[i] = ZD_table_[src[i]];
    depth[i + 1] = depth[i];
  }
}

MYNTEYE_END_NAMESPACE
