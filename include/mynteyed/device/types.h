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
#ifndef MYNTEYE_DEVICE_TYPES_H_
#define MYNTEYE_DEVICE_TYPES_H_
#pragma once

#include <cstdint>
#include <ostream>

#include "mynteyed/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

/**
 * @ingroup enumerations
 * @brief List device modes.
 *
 * Control the color & depth streams enabled or not.
 *
 * Note: y: available, n: unavailable, -: depends on StreamMode
 */
enum class DeviceMode : std::int32_t {
  DEVICE_COLOR = 0,  /**< IMAGE_LEFT_COLOR y IMAGE_RIGHT_COLOR - IMAGE_DEPTH n */
  DEVICE_DEPTH = 1,  /**< IMAGE_LEFT_COLOR n IMAGE_RIGHT_COLOR n IMAGE_DEPTH y */
  DEVICE_ALL = 2,    /**< IMAGE_LEFT_COLOR y IMAGE_RIGHT_COLOR - IMAGE_DEPTH y */
};

/**
 * @ingroup enumerations
 * @brief List color modes.
 */
enum class ColorMode : std::int32_t {
  COLOR_RAW       = 0,  /**< color raw */
  COLOR_RECTIFIED = 1,  /**< color rectified */
  COLOR_MODE_LAST
};

/**
 * @ingroup enumerations
 * @brief List depth modes.
 */
enum class DepthMode : std::int32_t {
  DEPTH_RAW      = 0,  /**< ImageFormat::DEPTH_RAW */
  DEPTH_GRAY     = 1,  /**< ImageFormat::DEPTH_GRAY_24 */
  DEPTH_COLORFUL = 2,  /**< ImageFormat::DEPTH_RGB */
  DEPTH_MODE_LAST
};

/**
 * @ingroup enumerations
 * @brief List stream modes.
 */
enum class StreamMode : std::int32_t {
  STREAM_640x480  = 0,  /**< 480p, vga, left */
  STREAM_1280x480 = 1,  /**< 480p, vga, left+right */
  STREAM_1280x720 = 2,  /**< 720p, hd, left */
  STREAM_2560x720 = 3,  /**< 720p, hd, left+right */
  STREAM_MODE_LAST
};

/**
 * @ingroup enumerations
 * @brief List stream formats.
 */
enum class StreamFormat : std::int32_t {
  STREAM_MJPG = 0,
  STREAM_YUYV = 1,
  STREAM_FORMAT_LAST
};

MYNTEYE_API
std::ostream& operator<<(std::ostream& os, const StreamFormat& code);

/**
 * @ingroup enumerations
 * @brief List image types.
 */
enum class ImageType : std::int32_t {
  /** LEFT Color. */
  IMAGE_LEFT_COLOR,
  /** RIGHT Color. */
  IMAGE_RIGHT_COLOR,
  /** Depth. */
  IMAGE_DEPTH,
  /** All. */
  IMAGE_ALL,
};

MYNTEYE_API
std::ostream& operator<<(std::ostream& os, const ImageType& code);

/**
 * @ingroup enumerations
 * @brief List image formats.
 */
enum class ImageFormat : std::int32_t {
  IMAGE_BGR_24,   /**< 8UC3 */
  IMAGE_RGB_24,   /**< 8UC3 */
  IMAGE_GRAY_8,   /**< 8UC1 */
  IMAGE_GRAY_16,  /**< 16UC1 */
  IMAGE_GRAY_24,  /**< 8UC3 */
  IMAGE_YUYV,     /**< 8UC2 */
  IMAGE_MJPG,
  // color
  COLOR_BGR   = IMAGE_BGR_24,  // > COLOR_RGB
  COLOR_RGB   = IMAGE_RGB_24,  // > COLOR_BGR
  COLOR_YUYV  = IMAGE_YUYV,    // > COLOR_BGR, COLOR_RGB
  COLOR_MJPG  = IMAGE_MJPG,    // > COLOR_BGR, COLOR_RGB
  // depth
  DEPTH_RAW     = IMAGE_GRAY_16,  // > DEPTH_GRAY
  DEPTH_GRAY    = IMAGE_GRAY_8,
  DEPTH_GRAY_24 = IMAGE_GRAY_24,
  DEPTH_BGR     = IMAGE_BGR_24,   // > DEPTH_RGB
  DEPTH_RGB     = IMAGE_RGB_24,   // > DEPTH_BGR
  /** Last guard. */
  IMAGE_FORMAT_LAST
};

/**
 * @ingroup enumerations
 * @brief SensorType types.
 */
enum class SensorType : std::int32_t {
  SENSOR_TYPE_H22 = 0,
  SENSOR_TYPE_OV7740,
  SENSOR_TYPE_AR0134,
  SENSOR_TYPE_AR0135,
  SENSOR_TYPE_OV9714
};

/**
 * @ingroup enumerations
 * @brief SensorMode modes.
 */
enum class SensorMode : std::int32_t {
  LEFT = 0,
  RIGHT,
  ALL
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_TYPES_H_
