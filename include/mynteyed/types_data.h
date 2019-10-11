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
#ifndef MYNTEYE_TYPES_DATA_H_
#define MYNTEYE_TYPES_DATA_H_
#pragma once

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mynteyed/device/image.h"
#include "mynteyed/stubs/types_calib.h"

MYNTEYE_BEGIN_NAMESPACE

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
  bool operator >(const ImgInfo &other) {
    if (frame_id > other.frame_id || timestamp >  other.timestamp) {
      return true;
    } else {
      return false;
    }
  }
  friend std::ostream &operator <<(std::ostream &os,
      const ImgInfo &m) {
    os << "frame_id: " << m.frame_id
       << "  timestamp:" << m.timestamp
       << std::endl;
    return os;
  }
};

#define MYNTEYE_IMU_ACCEL 1
#define MYNTEYE_IMU_GYRO 2

/**
 * @ingroup datatypes
 * @brief Imu data
 */
struct MYNTEYE_API ImuData {
  /**
   * Data type
   *   MYNTEYE_IMU_ACCEL: accelerometer
   *   MYNTEYE_IMU_GYRO: gyroscope
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

/**
 * @ingroup datatypes
 * GPS data.
 */
struct MYNTEYE_API GPSData {
  std::uint64_t device_time;
  double latitude;
  double longitude;
  std::uint64_t latitude_degree;
  std::uint64_t latitude_cent;
  std::uint64_t latitude_second;
  std::uint64_t longitude_degree;
  std::uint64_t longitude_cent;
  std::uint64_t longitude_second;
  std::uint8_t NS;
  std::uint8_t EW;

  std::uint16_t year;
  std::uint8_t month;
  std::uint8_t day;
  std::uint8_t hour;
  std::uint8_t minute;
  std::uint8_t second;
};

/**
 * @ingroup datatypes
 * ObstacleDistance data.
 */
struct MYNTEYE_API ObstacleDis {
  std::uint64_t detection_time;
  /* unit (mm) */
  std::uint16_t distance;
};

/**
 * @ingroup datatypes
 * Stream data.
 */
struct MYNTEYE_API StreamData {
  /** Image data */
  std::shared_ptr<Image> img;
  /** Image information */
  std::shared_ptr<ImgInfo> img_info;

  bool operator==(const StreamData& other) const {
    if (img_info && other.img_info) {
      return img_info->frame_id == other.img_info->frame_id &&
             img_info->timestamp == other.img_info->timestamp;
    }
    return false;
  }
};

/**
 * @ingroup datatypes
 * Motion data.
 */
struct MYNTEYE_API MotionData {
  /** ImuData. */
  std::shared_ptr<ImuData> imu;

  bool operator==(const MotionData &other) const {
    if (imu && other.imu) {
      return imu->flag == other.imu->flag &&
             imu->timestamp == other.imu->timestamp;
    }
    return false;
  }
};

/**
 * @ingroup datatypes
 * Location data.
 */
struct MYNTEYE_API LocationData {
  /** GPSData */
  std::shared_ptr<GPSData> gps;

  bool operator==(const LocationData &other) const {
    if (gps && other.gps) {
      return gps->device_time == other.gps->device_time;
    }

    return false;
  }
};

/**
 * @ingroup datatypes
 * Distance data.
 */
struct MYNTEYE_API DistanceData {
  /** ObstacleDis */
  std::shared_ptr<ObstacleDis> dis;

  bool operator==(const DistanceData &other) const {
    if (dis && other.dis) {
      return dis->detection_time ==
        other.dis->detection_time;
    }

    return false;
  }
};

#define MYNTEYE_PROPERTY(TYPE, NAME) \
 public:                             \
  void set_##NAME(TYPE NAME) { NAME##_ = NAME; } \
  TYPE NAME() const { return NAME##_; }          \
 private:       \
  TYPE NAME##_; \

/**
 * Version.
 */
class MYNTEYE_API Version {
 public:
  using size_t = std::size_t;
  using value_t = std::uint8_t;

  Version() = default;
  Version(value_t major, value_t minor) : major_(major), minor_(minor) {}
  explicit Version(const std::string &name)
      : major_(parse_part(name, 0)), minor_(parse_part(name, 1)) {}
  virtual ~Version() {}

  bool operator==(const Version &other) const {
    return major_ == other.major_ && minor_ == other.minor_;
  }
  bool operator<=(const Version &other) const {
    if (major_ < other.major_)
      return true;
    if (major_ > other.major_)
      return false;
    return minor_ <= other.minor_;
  }
  bool operator!=(const Version &other) const {
    return !(*this == other);
  }
  bool operator<(const Version &other) const {
    return !(*this == other) && (*this <= other);
  }
  bool operator>(const Version &other) const {
    return !(*this <= other);
  }
  bool operator>=(const Version &other) const {
    return (*this == other) || (*this > other);
  }
  bool is_between(const Version &from, const Version &until) {
    return (from <= *this) && (*this <= until);
  }

  std::string to_string() const;

  static std::vector<std::string> split(const std::string &s);
  static value_t parse_part(const std::string &name, size_t part);

  MYNTEYE_PROPERTY(value_t, major)
  MYNTEYE_PROPERTY(value_t, minor)
};

/**
 * Hardware version.
 */
class MYNTEYE_API HardwareVersion : public Version {
 public:
  using flag_t = std::bitset<8>;

  HardwareVersion() = default;
  HardwareVersion(value_t major, value_t minor, value_t flag = 0)
      : Version(major, minor), flag_(flag) {}
  explicit HardwareVersion(const std::string &name, value_t flag = 0)
      : Version(parse_part(name, 0), parse_part(name, 1)), flag_(flag) {}

  MYNTEYE_PROPERTY(flag_t, flag)
};

/**
 * Type.
 */
class MYNTEYE_API Type {
 public:
  using size_t = std::size_t;
  using value_t = std::uint16_t;

  Type() = default;
  Type(value_t vendor, value_t product) : vendor_(vendor), product_(product) {}
  explicit Type(const std::string &name)
      : vendor_(parse_part(name, 0, 2)), product_(parse_part(name, 2, 2)) {}
  virtual ~Type() {}

  std::string to_string() const;
  static value_t parse_part(const std::string &name, size_t pos, size_t count);

  MYNTEYE_PROPERTY(value_t, vendor)
  MYNTEYE_PROPERTY(value_t, product)
};

namespace device {

/**
 * @ingroup datatypes
 * Device descriptors.
 */
struct MYNTEYE_API Descriptors {
  bool ok;
  std::string name;
  std::string serial_number;
  Version firmware_version;
  HardwareVersion hardware_version;
  Version spec_version;
  Type lens_type;
  Type imu_type;
  std::uint16_t nominal_baseline;
};

/**
 * @ingroup datatypes
 * Device imu paramters.
 */
struct MYNTEYE_API ImuParams {
  bool ok;
  ImuIntrinsics in_accel;
  ImuIntrinsics in_gyro;
  Extrinsics ex_left_to_imu;
};

}  // namespace device

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TYPES_DATA_H_
