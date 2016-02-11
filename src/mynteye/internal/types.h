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
#ifndef MYNTEYE_INTERNAL_TYPES_H_  // NOLINT
#define MYNTEYE_INTERNAL_TYPES_H_
#pragma once

#include <cstdint>

#include <array>
#include <bitset>
#include <string>
#include <vector>

#include "mynteye/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

/**
 * @ingroup datatypes
 * Image packet.
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
 * Image response packet
 */
#pragma pack(push, 1)
struct ImgInfoResPacket {
  std::vector<ImgInfoPacket> packets;

  ImgInfoResPacket() = default;
  explicit ImgInfoResPacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    ImgInfoPacket packet(data);
    packets.push_back(packet);
  }
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * Imu segment.
 */
#pragma pack(push, 1)
struct ImuSegment {
  std::uint8_t flag;
  std::uint32_t timestamp;
  std::int16_t temperature;
  std::int16_t accel_or_gyro[3];

  ImuSegment() = default;
  explicit ImuSegment(std::uint8_t *data) {
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

/**
 * @ingroup datatypes
 * Imu packet.
 */
#pragma pack(push, 1)
struct ImuPacket {
  std::vector<ImuSegment> segments;

  ImuPacket() = default;
  explicit ImuPacket(std::uint8_t *data) {
    from_data(data);
  }
  void from_data(std::uint8_t *data) {
    segments.push_back(ImuSegment(data));
  }
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * Imu response packet.
 */
#pragma pack(push, 1)
struct ImuResPacket {
  std::uint8_t header;
  std::vector<ImuPacket> packets;

  ImuResPacket() = default;
  explicit ImuResPacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    ImuPacket packet(data);
    packets.push_back(packet);
  }
  void from_header_data(std::uint8_t *data) {
    header = *data | *(data + 1) << 8;
  }
};
#pragma pack(pop)

MYNTEYE_END_NAMESPACE

#endif //MYNTEYE_INTERNAL_TYPES_H_ // NOLINT
