#ifndef MYNTEYE_DEVICE_COLORIZER_LINUX_H_
#define MYNTEYE_DEVICE_COLORIZER_LINUX_H_
#pragma once

#include "mynteyed/device/colorizer_p.h"

#include "color_palette_generator.h"

MYNTEYE_BEGIN_NAMESPACE

class ColorizerLinux : public ColorizerPrivate {
 public:
  ColorizerLinux() = default;
  virtual ~ColorizerLinux() = default;

  void Init(float z14_far,
      bool is_8bits,
      std::shared_ptr<CameraCalibration> calib_params) override;

  Image::pointer Process(const Image::pointer& depth_buf,
        const DepthMode& depth_mode) override;

 private:
  RGBQUAD m_ColorPalette[256];
  RGBQUAD m_GrayPalette[256];
  RGBQUAD m_ColorPaletteD11[2048];
  RGBQUAD m_GrayPaletteD11[2048];
  RGBQUAD m_ColorPaletteZ14[16384];
  RGBQUAD m_GrayPaletteZ14[16384];
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_COLORIZER_LINUX_H_
