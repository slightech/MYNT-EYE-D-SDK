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
#ifndef MYNTEYE_TYPES_H_
#define MYNTEYE_TYPES_H_
#pragma once

#include <cstdint>
#include <ostream>

#include "mynteye/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

/**
 * @ingroup enumerations
 * @brief List error codes.
 */
enum ErrorCode {
  /** Standard code for successful behavior. */
  SUCCESS = 0,
  /** Standard code for unsuccessful behavior. */
  ERROR_FAILURE,
  /**
   * File cannot be opened for not exist, not a regular file or
   * any other reason.
   */
  ERROR_FILE_OPEN_FAILED,
  /** Camera cannot be opened for not plugged or any other reason. */
  ERROR_CAMERA_OPEN_FAILED,
  /** Camera is not opened now. */
  ERROR_CAMERA_NOT_OPENED,
  /** Camera retrieve the image failed. */
  ERROR_CAMERA_RETRIEVE_FAILED,
  /** Last guard. */
  ERROR_LAST
};

/**
 * @ingroup enumerations
 * @brief List view types.
 */
enum ViewType {
  /** Image. */
  VIEW_IMAGE,
  /** Depth. */
  VIEW_DEPTH,
  /** Last guard. */
  VIEW_LAST
};

/**
 * @ingroup enumerations
 * @brief List device types.
 */
enum DeviceType {
  DEVICE_OTHERS = 0,
  DEVICE_AXES1,
  DEVICE_PUMA,
  DEVICE_KIWI,
  DEVICE_LAST
};

/**
 * @ingroup enumerations
 * @brief List depth modes.
 */
enum DepthMode {
  DEPTH_NON,
  DEPTH_GRAY,
  DEPTH_COLORFUL,
  DEPTH_NON_16UC1,
  DEPTH_NON_8UC1,
  DEPTH_LAST
};


/**
 * @ingroup enumerations
 * @brief List stream mode.
 */
enum StreamMode {
  STREAM_1280x720,
  STREAM_2560x720,
  // STREAM_2560x960,
  STREAM_1280x480,
  STREAM_640x480,
  STREAM_MODE_LAST
};

/**
 * @ingroup enumerations
 * @brief List stream format.
 */
enum StreamFormat {
  STREAM_MJPG,
  STREAM_YUYV,
  STREAM_FORMAT_LAST
};

MYNTEYE_END_NAMESPACE

MYNTEYE_API
std::ostream& operator<<(std::ostream& os, const mynteye::StreamFormat& code);

#endif  // MYNTEYE_TYPES_H_
