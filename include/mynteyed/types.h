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

#include "mynteyed/device/types.h"
#include "mynteyed/types_data.h"

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
 * @brief The descriptor fields.
 */
enum class Descriptor : std::int32_t {
  /** Device name */
  DEVICE_NAME,
  /** Serial number */
  SERIAL_NUMBER,
  /** Firmware version */
  FIRMWARE_VERSION,
  /** Hardware version */
  HARDWARE_VERSION,
  /** Spec version */
  SPEC_VERSION,
  /** Lens type */
  LENS_TYPE,
  /** IMU type */
  IMU_TYPE,
  /** Nominal baseline */
  NOMINAL_BASELINE,
  /** Last guard */
  DESC_LAST
};

/**
 * @ingroup enumerations
 * @brief Process modes.
 */
enum class ProcessMode : std::int32_t {
  PROC_NONE           = 0,
  PROC_IMU_ASSEMBLY   = 1,
  PROC_IMU_TEMP_DRIFT = 2,
  PROC_IMU_ALL        = PROC_IMU_ASSEMBLY | PROC_IMU_TEMP_DRIFT
};

inline
std::int32_t operator&(const std::int32_t& lhs, const ProcessMode& rhs) {
  return lhs & static_cast<std::int32_t>(rhs);
}
inline
std::int32_t operator&(const ProcessMode& lhs, const std::int32_t& rhs) {
  return static_cast<std::int32_t>(lhs) & rhs;
}
inline
std::int32_t operator&(const ProcessMode& lhs, const ProcessMode& rhs) {
  return static_cast<std::int32_t>(lhs) & static_cast<std::int32_t>(rhs);
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TYPES_H_
