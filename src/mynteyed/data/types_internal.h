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
#ifndef MYNTEYE_DATA_TYPES_INTERNAL_H_
#define MYNTEYE_DATA_TYPES_INTERNAL_H_
#pragma once

#include <cstdint>
#include <vector>

#include "mynteyed/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

/**
 * @ingroup datatypes
 * Image info packet.
 */
#pragma pack(push, 1)
struct ImgInfoPacket {
  std::uint16_t frame_id;
  std::uint32_t timestamp;
  std::uint16_t exposure_time;

  ImgInfoPacket() = default;
  explicit ImgInfoPacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    timestamp = (*(data + 2)) | (*(data + 3) << 8) | (*(data + 4) << 16) |
                (*(data + 5) << 24);
    frame_id = (*(data + 6)) | (*(data + 7) << 8);
    exposure_time = (*(data + 8)) | (*(data + 9) << 8);
  }
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * Imu data packet.
 */
#pragma pack(push, 1)
struct ImuDataPacket {
  std::uint8_t flag;
  std::uint32_t timestamp;
  std::int16_t temperature;
  std::int16_t accel_or_gyro[3];

  ImuDataPacket() = default;
  explicit ImuDataPacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    flag = *data + 1;
    timestamp =
        *(data + 2) | *(data + 3) << 8 | *(data + 4) << 16 | *(data + 5) << 24;
    accel_or_gyro[0] = *(data + 6) | *(data + 7) << 8;
    accel_or_gyro[1] = *(data + 8) | *(data + 9) << 8;
    accel_or_gyro[2] = *(data + 10) | *(data + 11) << 8;
    temperature = *(data + 12) | *(data + 13) << 8;
  }
};
#pragma pack(pop)

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DATA_TYPES_INTERNAL_H_
