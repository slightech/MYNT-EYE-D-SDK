#ifndef MYNTEYE_DEVICE_COLORIZER_WIN_H_
#define MYNTEYE_DEVICE_COLORIZER_WIN_H_
#pragma once

#include <Windows.h>

#include "mynteyed/device/colorizer.h"

MYNTEYE_BEGIN_NAMESPACE

class ColorizerWin : public Colorizer {
 public:
  ColorizerWin() = default;
  virtual ~ColorizerWin() = default;

  void Init(float z14_far,
      bool is_8bits,
      std::shared_ptr<CameraCalibration> calib_params) override;

  Image::pointer Process(const Image::pointer& depth_buf,
        const DepthMode& depth_mode) override;

 private:
  void OnInit(float z14_far);

  unsigned char m_ColorPalette[256][4];
  unsigned char m_GrayPalette[256][4];
  RGBQUAD m_ColorPaletteD11[2048];
  RGBQUAD m_GrayPaletteD11[2048];
  RGBQUAD m_ColorPaletteZ14[16384];
  RGBQUAD m_GrayPaletteZ14[16384];

  RGBQUAD color_palette_z14_[16384];
  RGBQUAD gray_palette_z14_[16384];
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_COLORIZER_WIN_H_
