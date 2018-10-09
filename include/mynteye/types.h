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
enum class ErrorCode : std::int32_t {
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
  /** Imu cannot be opened for not plugged or any other reason. */
  ERROR_IMU_OPEN_FAILED,
  /** Imu receive data timeout */
  ERROR_IMU_RECV_TIMEOUT,
  /** Imu receive data error */
  ERROR_IMU_DATA_ERROR,
  /** Last guard. */
  ERROR_CODE_LAST
};

/**
 * @ingroup enumerations
 * @brief Source allows the user to choose which data to be captured.
 */
enum class Source : std::uint8_t {
  /** Video streaming of stereo, color, depth, etc. */
  VIDEO_STREAMING,
  /** Motion tracking of IMU (accelerometer, gyroscope) */
  MOTION_TRACKING,
  /** Enable everything together */
  ALL,
  /** Last guard */
  LAST
};

/**
 * @ingroup enumerations
 * @brief List image types.
 */
enum class ImageType : std::int32_t {
  /** Color. */
  IMAGE_COLOR,
  /** Depth. */
  IMAGE_DEPTH,
  /** Last guard. */
  IMAGE_TYPE_LAST
};

/**
 * @ingroup enumerations
 * @brief List image formats.
 */
enum class ImageFormat : std::int32_t {
  IMAGE_BGR_24,   // 8UC3
  IMAGE_RGB_24,   // 8UC3
  IMAGE_GRAY_8,   // 8UC1
  IMAGE_GRAY_16,  // 16UC1
  IMAGE_GRAY_24,  // 8UC3
  IMAGE_YUYV,     // 8UC2
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
 * @brief List depth modes.
 */
enum class DepthMode : std::int32_t {
  DEPTH_RAW      = 0,  // ImageFormat::DEPTH_RAW
  DEPTH_GRAY     = 1,  // ImageFormat::DEPTH_GRAY_24
  DEPTH_COLORFUL = 2,  // ImageFormat::DEPTH_RGB
  DEPTH_MODE_LAST
};

/**
 * @ingroup enumerations
 * @brief List stream mode.
 */
enum class StreamMode : std::int32_t {
  STREAM_1280x720 = 0,
  STREAM_2560x720 = 1,
  STREAM_1280x480 = 2,
  STREAM_640x480  = 3,
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

MYNTEYE_END_NAMESPACE

MYNTEYE_API
std::ostream& operator<<(std::ostream& os, const mynteye::StreamFormat& code);

/**
 * @ingroup datatypes
 * @brief Image information
 */
struct MYNTEYE_API ImgInfo {
  /** Image frame id */
  std::uint16_t frame_id;

  /** Image timestamp */
  std::uint32_t timestamp;

  /** Image exposure time */
  std::uint16_t exposure_time;

  void Reset() {
    frame_id = 0;
    timestamp = 0;
    exposure_time = 0;
  }

  ImgInfo() {
    Reset();
  }
  ImgInfo(const ImgInfo &other) {
    frame_id = other.frame_id;
    timestamp = other.timestamp;
    exposure_time = other.exposure_time;
  }
  ImgInfo &operator=(const ImgInfo &other) {
    frame_id = other.frame_id;
    timestamp = other.timestamp;
    exposure_time = other.exposure_time;
    return *this;
  }
};

/**
 * @ingroup datatypes
 * @brief Imu data
 */
struct MYNTEYE_API ImuData {
  /**
   * Data type
   * 1: accelerometer
   * 2: gyroscope
   * */
  std::uint8_t flag;

  /** Imu gyroscope or accelerometer or frame timestamp */
  std::uint64_t timestamp;

  /** temperature */
  double temperature;

  /** Imu accelerometer data for 3-axis: X, Y, X. */
  double accel[3];

  /** Imu gyroscope data for 3-axis: X, Y, Z. */
  double gyro[3];

  void Reset() {
    flag = 0;
    timestamp = 0;
    temperature = 0;
    std::fill(accel, accel + 3, 0);
    std::fill(gyro, gyro + 3, 0);
  }

  ImuData() {
    Reset();
  }
};

#endif  // MYNTEYE_TYPES_H_
