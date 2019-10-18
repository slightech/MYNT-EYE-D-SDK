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

#include <string.h>
#include <cstdint>
#include <vector>
#include <iostream>

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
  float temperature;
  float accel[3];
  float gyro[3];

  ImuDataPacket() = default;
  explicit ImuDataPacket(bool is_v2, std::uint8_t *data) {
    if(is_v2)
      from_data_v2(data);
    else
      from_data_v1(data);
  }

  void from_data_v1(std::uint8_t *data) {
    std::uint16_t temp;
    std::uint16_t accel_or_gyro[3];
    flag = *data + 1;
    timestamp =
        *(data + 2) | *(data + 3) << 8 | *(data + 4) << 16 | *(data + 5) << 24;
    accel_or_gyro[0] = *(data + 6) | *(data + 7) << 8;
    accel_or_gyro[1] = *(data + 8) | *(data + 9) << 8;
    accel_or_gyro[2] = *(data + 10) | *(data + 11) << 8;
    temp = *(data + 12) | *(data + 13) << 8;
    temperature = temp * 0.125 + 23;
    if (flag == 1) {
      accel[0] = accel_or_gyro[0] * 12.f / 0x10000;
      accel[1] = accel_or_gyro[1] * 12.f / 0x10000;
      accel[2] = accel_or_gyro[2] * 12.f / 0x10000;
      gyro[0] = 0.f;
      gyro[1] = 0.f;
      gyro[2] = 0.f;
    } else if (flag == 2) {
      accel[0] = 0.f;
      accel[1] = 0.f;
      accel[2] = 0.f;
      gyro[0] = accel_or_gyro[0] * 2000.f / 0x10000;
      gyro[1] = accel_or_gyro[1] * 2000.f / 0x10000;
      gyro[2] = accel_or_gyro[2] * 2000.f / 0x10000;
    } else {
      accel[0] = 0.f;
      accel[1] = 0.f;
      accel[2] = 0.f;
      gyro[0] = 0.f;
      gyro[1] = 0.f;
      gyro[2] = 0.f;
    }
  }

  void from_data_v2(std::uint8_t *data) {
    flag = *data + 1;
    timestamp =
        *(data + 2) | *(data + 3) << 8 | *(data + 4) << 16 | *(data + 5) << 24;
    if (flag == 1) {
      accel[0] = *((float*)(data + 6));
      accel[1] = *((float*)(data + 10));
      accel[2] = *((float*)(data + 14));
      gyro[0] = 0.f;
      gyro[1] = 0.f;
      gyro[2] = 0.f;
    } else if (flag == 2) {
      accel[0] = 0.f;
      accel[1] = 0.f;
      accel[2] = 0.f;
      gyro[0] = *((float*)(data + 6));
      gyro[1] = *((float*)(data + 10));
      gyro[2] = *((float*)(data + 14));
    } else if (flag == 10) {
      accel[0] = *((float*)(data + 6));
      accel[1] = *((float*)(data + 10));
      accel[2] = *((float*)(data + 14));
      gyro[0] = *((float*)(data + 18));
      gyro[1] = *((float*)(data + 22));
      gyro[2] = *((float*)(data + 26));
    } else {
      accel[0] = 0.f;
      accel[1] = 0.f;
      accel[2] = 0.f;
      gyro[0] = 0.f;
      gyro[1] = 0.f;
      gyro[2] = 0.f;
    }

  }
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * GPS data packet.
 */
#pragma pack(push, 1)
struct GPSDataPacket {
  std::uint8_t flag;
  std::uint64_t device_time;
  double latitude;
  double longitude;
  std::uint8_t NS;
  std::uint8_t EW;

  std::uint16_t year;
  std::uint8_t month;
  std::uint8_t day;
  std::uint8_t hour;
  std::uint8_t minute;
  std::uint8_t second;

  GPSDataPacket() = default;
  explicit GPSDataPacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    flag = *data + 1;
    device_time = *(data + 2) | *(data + 3) << 8 |
      *(data + 4) << 16 | *(data + 5) << 24;
    hour = *(data + 6);
    minute = *(data + 7);
    second = *(data + 8);
    year = *(data + 9) | *(data + 10) << 8;
    month = *(data + 11);
    day = *(data + 12);
    NS = *(data + 13);
    memcpy(&latitude, data + 14, 8);
    EW = *(data + 22);
    memcpy(&longitude, data + 23, 8);
  }
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * ObstacleDistance data packet.
 */
#pragma pack(push, 1)
struct ObstacleDisPacket {
  std::uint8_t flag;
  std::uint64_t detection_time;
  std::uint16_t distance;

  ObstacleDisPacket() = default;
  explicit ObstacleDisPacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    flag = *data + 1;
    detection_time = *(data + 2) | *(data + 3) << 8 |
      *(data + 4) << 16 | *(data + 5) << 24;
    distance = *(data + 6) | *(data + 7) << 8;
  }
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * temperature packet.
 */
#pragma pack(push, 1)
struct TemperaturePacket {
  std::uint32_t timestamp;
  std::uint32_t temperature;

  TemperaturePacket() = default;
  explicit TemperaturePacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    timestamp =
        *(data + 2) | *(data + 3) << 8 | *(data + 4) << 16 | *(data + 5) << 24;
    temperature = *((float*)(data + 6));
  }
};
#pragma pack(pop)

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DATA_TYPES_INTERNAL_H_
