// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef MYNTEYE_DEVICE_OPEN_PARAMS_H_
#define MYNTEYE_DEVICE_OPEN_PARAMS_H_
#pragma once

#include <string>

#include "mynteye/device/types.h"
#include "mynteye/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

/**
 * Device open parameters.
 */
struct MYNTEYE_API OpenParams {
  /**
   * Device index.
   */
  std::int32_t dev_index;

  /**
   * Framerate, range [0,60], [0,30](STREAM_2560x720), default 10.
   */
  std::int32_t framerate;

  /**
   * Color mode, default COLOR_RAW.
   */
  ColorMode color_mode;

  /**
   * Depth mode, default DEPTH_COLORFUL.
   */
  DepthMode depth_mode;

  /**
   * Stream mode of color & depth, default STREAM_1280x720.
   */
  StreamMode stream_mode;

  /**
   * Stream format of color, default STREAM_YUYV.
   */
  StreamFormat color_stream_format;

  /**
   * Stream format of depth, default STREAM_YUYV.
   */
  StreamFormat depth_stream_format;

  /**
   * Device mode, default ALL
   * Note:: If DEVICE_COLOR is enable, ImageType::IMAGE_DEPTH will not be available.
   *        if DEVICE_DEPTH is enable, ImageType::IMAGE_LEFT_COLOR and
   *           ImageType::IMAGE_RIGHT_COLOR will not be available.
   */
  DeviceMode device_mode;

  /**
   * Auto-exposure, default true.
   */
  bool state_ae;

  /**
   * Auto-white balance, default true.
   */
  bool state_awb;

  /**
   * IR (Infrared), range [0,6], default 0.
   */
  std::uint8_t ir_intensity;

  /** Constructor. */
  OpenParams();
  explicit OpenParams(const std::int32_t& dev_index);

  /** Destructor. */
  ~OpenParams();
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_OPEN_PARAMS_H_
