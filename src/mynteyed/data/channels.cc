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
#include "mynteyed/data/channels.h"

#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <iterator>
#include <chrono>
#include <stdexcept>

#include "mynteyed/data/hid/hid.h"
#include "mynteyed/util/log.h"
#include "mynteyed/util/strings.h"

// #define PACKET_PRINT
// #define PACKET_STAMP_DETECTION

#define PACKET_SIZE 64
#define DATA_SIZE 15

MYNTEYE_BEGIN_NAMESPACE

namespace {

inline std::uint8_t check_sum(std::uint8_t *buf, std::uint16_t length) {
  std::uint8_t crc8 = 0;
  while (length--) {
    crc8 = crc8 ^ (*buf++);
  }
  return crc8;
}

void CheckSpecVersion(const Version *spec_version) {
  if (spec_version == nullptr) {
    LOGE("%s %d:: Spec version must be specified.", __FILE__, __LINE__);
    return;
  }

  std::vector<std::string> spec_versions{"1.0"};
  for (auto &&spec_ver : spec_versions) {
    if (*spec_version == Version(spec_ver)) {
      return;  // supported
    }
  }

  std::ostringstream ss;
  std::copy(
      spec_versions.begin(), spec_versions.end(),
      std::ostream_iterator<std::string>(ss, ","));
  LOGE("%s %d:: Spec version %s not supported, must in [%s]",
      __FILE__, __LINE__, spec_version->to_string().c_str(),
      ss.str().c_str());
}

}  // namespace

Channels::Channels() : img_callback_(nullptr),
  imu_callback_(nullptr),
  gps_callback_(nullptr),
  dis_callback_(nullptr) {
  hid_ = std::make_shared<hid::hid_device>();
  Detect();
  Open();
}

Channels::~Channels() {
  if (is_hid_tracking_) {
    StopHidTracking();
  }
  Close();
}

bool Channels::IsAvaliable() const {
  return IsHidAvaliable();
}

bool Channels::IsOpened() const {
  return IsHidOpened();
}

void Channels::SetImuDataCallback(imu_callback_t callback) {
  imu_callback_ = callback;
}

void Channels::SetImgInfoCallback(img_callback_t callback) {
  img_callback_ = callback;
}

void Channels::SetGPSDataCallback(gps_callback_t callback) {
  gps_callback_ = callback;
}

void Channels::SetDisDataCallback(dis_callback_t callback) {
  dis_callback_ = callback;
}

bool Channels::IsHidAvaliable() const {
  return is_hid_exist_;
}

bool Channels::IsHidOpened() const {
  return is_hid_opened_;
}

bool Channels::IsHidTracking() const {
  return is_hid_tracking_;
}

bool Channels::StartHidTracking() {
  if (!is_hid_opened_) {
    LOGW("WARNING:: hid device was not opened.");
    return false;
  }
  if (is_hid_tracking_) {
    LOGI("INFO:: hid device was tracking already.");
    return true;
  }

  is_hid_tracking_ = true;
  hid_track_thread_ = std::thread([this]() {
    while (is_hid_tracking_) {
      DoHidTrack();
    }
  });

  if (is_hid_tracking_)
    return true;
  else
    return false;
}

bool Channels::StopHidTracking() {
  if (!is_hid_opened_) {
    LOGW("WARNING:: hid device was not opened.");
    return false;
  }
  if (!is_hid_tracking_) {
    LOGI("INFO:: hid device was not tracking already.");
    return true;
  }

  is_hid_tracking_ = false;
  if (hid_track_thread_.joinable()) {
    hid_track_thread_.join();
  }
  return true;
}

void Channels::Detect() {
  DetectHid();
}

bool Channels::Open() {
  return OpenHid();
}

void Channels::Close() {
  CloseHid();
}

void Channels::DetectHid() {
  is_hid_exist_ = hid_->find_device();
  // LOGI("is_hid_exist_: %s", (is_hid_exist_ ? "true" : "false"));
}

bool Channels::OpenHid() {
  if (is_hid_opened_) {
    return true;
  }
  if (is_hid_exist_) {
    if (hid_->open(1, -1, -1) < 0) {
      if (hid_->open(1, -1, -1) < 0) {
        LOGE("%s, %d:: Open device failed, You must first execute "
            "the \"make init\" command.", __FILE__, __LINE__);
        return false;
      }
    }
    is_hid_opened_ = true;
  }
  return is_hid_opened_;
}

void Channels::CloseHid() {
  if (!is_hid_opened_) {
    return;
  }
  hid_->close(0);
  is_hid_opened_ = false;
}

bool Channels::DoHidTrack() {
  static imu_packets_t imu_packets;
  static img_packets_t img_packets;
  static gps_packets_t gps_packets;
  static dis_packets_t dis_packets;
  imu_packets.clear();
  img_packets.clear();
  gps_packets.clear();
  dis_packets.clear();

  if (!DoHidDataExtract(imu_packets, img_packets,
        gps_packets, dis_packets)) {
    return false;
  }

  if (imu_callback_) {
    for (auto &&imu_packet : imu_packets) {
      imu_callback_(imu_packet);
    }
  }
  if (img_callback_) {
    for (auto &&img_packet : img_packets) {
      img_callback_(img_packet);
    }
  }
  if (gps_callback_) {
    for (auto &&gps_packet : gps_packets) {
      gps_callback_(gps_packet);
    }
  }
  if (dis_callback_) {
    for (auto &&dis_packet : dis_packets) {
      dis_callback_(dis_packet);
    }
  }

  return true;
}

#ifdef PACKET_PRINT
void print_imu_data(const ImuDataPacket& imu_data) {
  std::cout << std::dec;
  if (imu_data.flag == MYNTEYE_IMU_ACCEL) {
    std::cout << "    [accel] stamp: " << imu_data.timestamp
      << ", x: " << imu_data.accel_or_gyro[0] * 12.f / 0x10000
      << ", y: " << imu_data.accel_or_gyro[1] * 12.f / 0x10000
      << ", z: " << imu_data.accel_or_gyro[2] * 12.f / 0x10000
      << ", temp: " << imu_data.temperature * 0.125 + 23
      << std::endl;
  } else if (imu_data.flag == MYNTEYE_IMU_GYRO) {
    std::cout << "    [gyro] stamp: " << imu_data.timestamp
      << ", x: " << imu_data.accel_or_gyro[0] * 2000.f / 0x10000
      << ", y: " << imu_data.accel_or_gyro[1] * 2000.f / 0x10000
      << ", z: " << imu_data.accel_or_gyro[2] * 2000.f / 0x10000
      << ", temp: " << imu_data.temperature * 0.125 + 23
      << std::endl;
  }
}

void print_img_info(const ImgInfoPacket& img_info) {
  std::cout << std::dec;
  std::cout << "    [img_info] fid: " << img_info.frame_id
      << ", stamp: " << img_info.timestamp
      << ", expos: " << img_info.exposure_time << std::endl;
}
#endif

#ifdef PACKET_STAMP_DETECTION
void detect_imu_data_stamp(const ImuDataPacket& imu_data) {
  static std::uint32_t accel_stamp_last = imu_data.timestamp;
  static std::uint32_t gyro_stamp_last = imu_data.timestamp;

  std::uint32_t stamp = imu_data.timestamp;
  std::uint32_t stamp_diff;
  if (imu_data.flag == MYNTEYE_IMU_ACCEL) {
    stamp_diff = stamp - accel_stamp_last;
    accel_stamp_last = stamp;
  } else {
    stamp_diff = stamp - gyro_stamp_last;
    gyro_stamp_last = stamp;
  }

  if (stamp_diff <= 0) {
    // same or rollback
    std::cout << "[STAMP_WARN]["
        << (imu_data.flag == MYNTEYE_IMU_ACCEL ? "accel" : "gyro") << "]"
        << " stamp: " << stamp
        << ", diff: " << stamp_diff << std::endl;
  }
}

void detect_img_info_stamp(const ImgInfoPacket& img_info) {
  static std::uint32_t stamp_last = img_info.timestamp;
  auto stamp = img_info.timestamp;
  auto stamp_diff = stamp - stamp_last;
  stamp_last = stamp;
  if (stamp_diff <= 0) {
    // same or rollback
    std::cout << "[STAMP_WARN][img_info] fid: " << img_info.frame_id
        << ", stamp: " << stamp
        << ", diff: " << stamp_diff << std::endl;
  }
}
#endif

bool Channels::DoHidDataExtract(imu_packets_t &imu, img_packets_t &img,
    gps_packets_t &gps, dis_packets_t &dis) {
  std::uint8_t data[PACKET_SIZE * 2]{};
  std::fill(data, data + PACKET_SIZE * 2, 0);

  int size = hid_->receive(0, data, PACKET_SIZE * 2, 220);
  if (size < 0) {
    // LOGE("%s, %d:: Failed to retrieve data. device is disconnected.", __FILE__, __LINE__);
    return false;
  }

  for (int i = 0; i < size / PACKET_SIZE; i++) {
    std::uint8_t *packet = data + i * PACKET_SIZE;

    if (packet[PACKET_SIZE - 1] !=
        check_sum(&packet[3], packet[2])) {
      LOGW("%s, %d:: Data is invaild, discarded.");
      continue;
    }

    auto sn = *packet | *(packet + 1) << 8;
    if (package_sn_ == sn) { continue; }
    package_sn_ = sn;

#ifdef PACKET_PRINT
    std::cout << std::endl;
    std::cout << "package[" << std::dec << sn << "]: ";
    std::copy(packet, packet + PACKET_SIZE, std::ostream_iterator<int>(
        std::cout << std::hex, " "));
    std::cout << std::endl;
#endif

    for (int offset = 3; offset <= PACKET_SIZE - DATA_SIZE;
        offset += DATA_SIZE) {
#ifdef PACKET_PRINT
      std::cout << "  data[" << static_cast<int>(*(packet + offset)) << "]: ";
      std::copy(packet + offset, packet + offset + DATA_SIZE,
          std::ostream_iterator<int>(std::cout << std::hex, " "));
      std::cout << std::endl;
#endif

      std::uint8_t header = *(packet + offset);
      if (header == ACCEL || header == GYRO) {
        auto&& imu_data = ImuDataPacket(packet + offset);
        imu.push_back(imu_data);
#ifdef PACKET_PRINT
        print_imu_data(imu_data);
#endif
#ifdef PACKET_STAMP_DETECTION
        detect_imu_data_stamp(imu_data);
#endif
      } else if (header == FRAME) {
        auto&& img_info = ImgInfoPacket(packet + offset);
        img.push_back(img_info);
#ifdef PACKET_PRINT
        print_img_info(img_info);
#endif
#ifdef PACKET_STAMP_DETECTION
        detect_img_info_stamp(img_info);
#endif
      } else if (header == DISTANCE) {
        auto&& obstacle_dis = ObstacleDisPacket(packet + offset);
        dis.push_back(obstacle_dis);
      } else if (header == LOCATION) {
        auto&& gps_data = GPSDataPacket(packet + offset);
        gps.push_back(gps_data);
        break;
      }
    }
  }

  return true;
}

namespace {

template <typename T>
T _from_data(const std::uint8_t *data) {
  std::size_t size = sizeof(T) / sizeof(std::uint8_t);
  T value = 0;
  for (std::size_t i = 0; i < size; i++) {
    // value |= data[i] << (8 * (size - i - 1));
    value |= data[i] << (8 * i);
  }
  return value;
}

template<>
double _from_data(const std::uint8_t *data) {
  return *(reinterpret_cast<const double *>(data));
}

std::string _from_data(const std::uint8_t *data, std::size_t count) {
  std::string s(reinterpret_cast<const char *>(data), count);
  strings::trim(s);
  return s;
}

std::size_t from_data(Channels::device_desc_t *desc, const std::uint8_t *data) {
  std::size_t i = 4;  // skip vid, pid
  // name, 20
  desc->name = _from_data(data + i, 20);
  i += 20;
  // serial_number, 24
  desc->serial_number = _from_data(data + i, 24);
  i += 24;
  // firmware_version, 2
  desc->firmware_version.set_minor(data[i]);
  desc->firmware_version.set_major(data[i + 1]);
  i += 2;
  // hardware_version, 3
  desc->hardware_version.set_minor(data[i]);
  desc->hardware_version.set_major(data[i + 1]);
  desc->hardware_version.set_flag(std::bitset<8>(data[i + 2]));
  i += 3;
  // spec_version, 2
  desc->spec_version.set_minor(data[i]);
  desc->spec_version.set_major(data[i + 1]);
  i += 2;
  // lens_type, 4
  desc->lens_type.set_vendor(_from_data<std::uint16_t>(data + i));
  desc->lens_type.set_product(_from_data<std::uint16_t>(data + i + 2));
  i += 4;
  // imu_type, 4
  desc->imu_type.set_vendor(_from_data<std::uint16_t>(data + i));
  desc->imu_type.set_product(_from_data<std::uint16_t>(data + i + 2));
  i += 4;
  // nominal_baseline, 2
  desc->nominal_baseline = _from_data<std::uint16_t>(data + i);
  i += 2;

  return i;
}

std::size_t from_data(ImuIntrinsics *in,
    const std::uint8_t *data, const Version *spec_version) {
  std::size_t i = 0;

  // scale
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      in->scale[j][k] = _from_data<double>(data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // assembly
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      in->assembly[j][k] = _from_data<double>(data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // drift
  for (std::size_t j = 0; j < 3; j++) {
    in->drift[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;
  // noise
  for (std::size_t j = 0; j < 3; j++) {
    in->noise[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;
  // bias
  for (std::size_t j = 0; j < 3; j++) {
    in->bias[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;
  i += 100;
  // temperature drift
  // x
  for (std::size_t j = 0; j < 2; j++) {
    in->x[j] = _from_data<double>(data + i + j * 8);
  }
  i += 16;
  // y
  for (std::size_t j = 0; j < 2; j++) {
    in->y[j] = _from_data<double>(data + i + j * 8);
  }
  i += 16;
  // z
  for (std::size_t j = 0; j < 2; j++) {
    in->z[j] = _from_data<double>(data + i + j * 8);
  }
  i += 16;

  UNUSED(spec_version);
  return i;
}

std::size_t from_data(Extrinsics *ex,
    const std::uint8_t *data, const Version *spec_version) {
  std::size_t i = 0;

  // rotation
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      ex->rotation[j][k] = _from_data<double>(data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // translation
  for (std::size_t j = 0; j < 3; j++) {
    ex->translation[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;

  UNUSED(spec_version);
  return i;
}

std::size_t from_data(
    Channels::imu_params_t *imu_params,
    const std::uint8_t *data,
    const Version *spec_version) {
  std::size_t i = 0;
  i += from_data(&imu_params->in_accel, data + i, spec_version);
  i += from_data(&imu_params->in_gyro, data + i, spec_version);
  i += from_data(&imu_params->ex_left_to_imu, data + i, spec_version);
  return i;
}

}  // namespace

bool Channels::PullFileData(bool device_desc,
    bool reserve,
    bool imu_params,
    std::uint8_t *data,
    std::uint16_t &file_size) {

  std::uint8_t buffer[64]{};

  buffer[0] = 0x0A;
  buffer[1] = 1;
  buffer[2] = 0x07 & ((device_desc << 0)
      | (reserve << 1) | (imu_params << 2));

  if (hid_->get_device_class() == 0xFF) {
    LOGE("%s %d:: Not support filechannel, please update firmware.",
        __FILE__, __LINE__);
    return false;
  }

  if (hid_->send(0, buffer, 64, 200) <= 0) {
    LOGE("%s %d:: Send command of imu instrinsics failed.",
        __FILE__, __LINE__);
    return false;
  }

  std::uint8_t req_count = 0;
  while (buffer[0] != 0x0B) {
    hid_->receive(0, buffer, 64, 2000);
    if (++req_count > 5) {
      LOGE("%s, %d:: Failed to retrieve data. Please update auxiliary firmware.", __FILE__, __LINE__);
      return false;
    }
  }

  std::uint32_t packets_sum = 0;
  std::int64_t packets_num = -1;
  std::uint32_t packets_index = 0;
  std::uint8_t *seek = data;
  while (true) {
    int ret = hid_->receive(0, buffer, 64, 220);
    if (ret <= 0) {
      LOGE("%s %d:: Require imu instrinsics failed.",
          __FILE__, __LINE__);
      return false;
    }

    if (buffer[0] == 0x0B && (packets_num == -1)) { continue; }
    if (packets_num == (buffer[0] | buffer[1] << 8)) { continue; }
    if (((buffer[0] | (buffer[1] << 8)) - packets_num) > 1) {
      LOGE("%s %d:: Lost index of %d packets, please retry. %ld",
          __FILE__, __LINE__, ((buffer[0] | buffer[1] << 8) - 1), packets_num);
      return false;
    }

    packets_num = buffer[0] | buffer[1] << 8;

    std::uint8_t length = buffer[2];
    if (length <= 0) { return false; }

    if (buffer[3 + length] != check_sum(&buffer[3], length)) {
      LOGE("%s %d:: Check error. please retry.",
          __FILE__, __LINE__);
      return false;
    }

    if (packets_num == 0) {
      packets_sum = 4 + (buffer[4] | (buffer[5] << 8));
      packets_index = 0;
    }
    packets_index += length;
    std::copy(buffer + 3, buffer + 3 + length, seek);
    seek += length;
    if (packets_index >= packets_sum) {
      file_size = packets_index;
      break;
    }
  }

  return true;
}

bool Channels::GetFiles(device_desc_t *desc,
    imu_params_t *imu_params, Version *spec_version) {
  if (desc == nullptr && imu_params == nullptr) {
    LOGE("%s %d:: Files are not provided to get.",
        __FILE__, __LINE__);
    return false;
  }

  std::uint8_t data[2000]{};
  std::uint16_t file_len;

  if (!PullFileData(true, true, true, data, file_len)) {
    LOGE("%s %d:: GetFiles failed.", __FILE__, __LINE__);
    return false;
  }

  std::uint16_t size = _from_data<std::uint16_t>(data + 1);
  std::uint8_t checksum = data[3 + size];

  std::uint8_t checksum_now = check_sum(data + 3, size);
  if (checksum != checksum_now) {
    LOGW("%s %d:: Files checksum should be %x, but %x now", __FILE__,
        __LINE__, static_cast<int>(checksum), static_cast<int>(checksum_now));
    return false;
  }

  Version *spec_ver = spec_version;
  std::size_t i = 3;
  std::size_t end = 3 + size;
  while (i < end) {
    std::uint8_t file_id = *(data + i);
    std::uint16_t file_size = _from_data<std::uint16_t>(data + i + 1);

    i += 3;
    switch (file_id) {
      case FID_DEVICE_DESC: {
        desc->ok = (file_size > 0 && from_data(desc, data + i) == file_size);
        if (!desc->ok) return false;
        spec_ver = &desc->spec_version;
        CheckSpecVersion(spec_ver);
      } break;
      case FID_RESERVE: break;
      case FID_IMU_PARAMS: {
        imu_params->ok = (file_size > 0);
        if (imu_params->ok) {
          CheckSpecVersion(spec_ver);
          if (from_data(imu_params, data + i, spec_version) != file_size) {
            imu_params->ok = false;
            return false;
          }
        }
      } break;
      default:
        LOGI("%s %d:: Unsupported file id: %u",
            __FILE__, __LINE__, file_id);
    }
    i += file_size;
  }

  return true;
}

namespace {

template <typename T>
std::size_t _to_data(T value, std::uint8_t *data) {
  std::size_t size = sizeof(T) / sizeof(std::uint8_t);
  for (std::size_t i = 0; i < size; i++) {
    data[i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xFF);
  }
  return size;
}

template <>
std::size_t _to_data(double value, std::uint8_t *data) {
  std::uint8_t *val = reinterpret_cast<std::uint8_t *>(&value);
  std::copy(val, val + 8, data);
  return 8;
}

std::size_t _to_data(std::string value, std::uint8_t *data, std::size_t count) {
  std::copy(value.begin(), value.end(), data);
  for (std::size_t i = value.size(); i < count; i++) {
    data[i] = ' ';
  }
  return count;
}

std::size_t to_data(
    const Channels::device_desc_t *desc, std::uint8_t *data,
    const Version *spec_version) {
  std::size_t i = 4;             // skip vid, pid
  // name, 20
  _to_data(desc->name, data + i, 20);
  i += 20;
  // serial_number, 24
  _to_data(desc->serial_number, data + i, 24);
  i += 24;
  // firmware_version, 2
  data[i] = desc->firmware_version.minor();
  data[i + 1] = desc->firmware_version.major();
  i += 2;
  // hardware_version, 3
  data[i] = desc->hardware_version.minor();
  data[i + 1] = desc->hardware_version.major();
  data[i + 2] =
      static_cast<std::uint8_t>(desc->hardware_version.flag().to_ulong());
  i += 3;
  // spec_version, 2
  data[i] = desc->spec_version.minor();
  data[i + 1] = desc->spec_version.major();
  i += 2;
  // lens_type, 4
  _to_data(desc->lens_type.vendor(), data + i);
  _to_data(desc->lens_type.product(), data + i + 2);
  i += 4;
  // imu_type, 4
  _to_data(desc->imu_type.vendor(), data + i);
  _to_data(desc->imu_type.product(), data + i + 2);
  i += 4;
  // nominal_baseline, 2
  _to_data(desc->nominal_baseline, data + i);
  i += 2;

  // others
  std::size_t size = i - 3;
  data[0] = Channels::FID_DEVICE_DESC;
  data[1] = static_cast<std::uint8_t>(size & 0xFF);
  data[2] = static_cast<std::uint8_t>((size >> 8) & 0xFF);

  UNUSED(spec_version);
  return size + 3;
}

std::size_t to_data(const ImuIntrinsics *in,
    std::uint8_t *data, const Version *spec_version) {
  std::size_t i = 0;

  // scale
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      _to_data(in->scale[j][k], data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // assembly
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      _to_data(in->assembly[j][k], data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // drift
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(in->drift[j], data + i + j * 8);
  }
  i += 24;
  // noise
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(in->noise[j], data + i + j * 8);
  }
  i += 24;
  // bias
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(in->bias[j], data + i + j * 8);
  }
  i += 24;
  i += 100;
  // temperature drift
  // x
  for (std::size_t j = 0; j < 2; j++) {
    _to_data<double>(in->x[j], data + i + j * 8);
  }
  i += 16;
  // y
  for (std::size_t j = 0; j < 2; j++) {
    _to_data<double>(in->y[j], data + i + j * 8);
  }
  i += 16;
  // z
  for (std::size_t j = 0; j < 2; j++) {
    _to_data<double>(in->z[j], data + i + j * 8);
  }
  i += 16;

  UNUSED(spec_version);
  return i;
}

std::size_t to_data(const Extrinsics *ex, std::uint8_t *data,
    const Version *spec_version) {
  std::size_t i = 0;

  // rotation
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      _to_data(ex->rotation[j][k], data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // translation
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(ex->translation[j], data + i + j * 8);
  }
  i += 24;

  UNUSED(spec_version);
  return i;
}

std::size_t to_data(
    const Channels::imu_params_t *imu_params,
    std::uint8_t *data,
    const Version *spec_version) {
  std::size_t i = 0;
  i += to_data(&imu_params->in_accel, data + i, spec_version);
  i += to_data(&imu_params->in_gyro, data + i, spec_version);
  i += to_data(&imu_params->ex_left_to_imu, data + i, spec_version);

  // others
  std::size_t size = i - 3;
  data[0] = Channels::FID_IMU_PARAMS;
  data[1] = static_cast<std::uint8_t>(size & 0xFF);
  data[2] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  return size + 3;
}

}  // namespace

bool Channels::PushFileData(
    std::uint8_t *data, std::uint16_t size) {
    std::uint8_t cmd[64];
  cmd[0] = 0x8A;
  cmd[1] = 4;
  cmd[2] = size & 0xFF;
  cmd[3] = (size & 0xFF00) >> 8;
  cmd[4] = (size & 0xFF0000) >> 16;
  cmd[5] = (size & 0xFF000000) >> 24;

  if (hid_->get_device_class() == 0xFF) {
    LOGE("%s %d:: Not support filechannel, please update firmware.",
        __FILE__, __LINE__);
    return false;
  }

  if (hid_->send(0, cmd, 64, 200) <= 0) {
    LOGE("%s %d:: Error reading, device not ready, retrying",
        __FILE__, __LINE__);
    return false;
  }

  std::uint8_t req_count = 0;
  while (0x8B != cmd[0]) {
    hid_->receive(0, cmd, 64, 2000);
    if (++req_count > 5) {
      // LOGE("%s %d:: Error reading, device went offline.",
      //     __FILE__, __LINE__);
      LOGE("%s, %d:: Failed to retrieve data. Please update auxiliary firmware.", __FILE__, __LINE__);
      return false;
    }
  }

  std::uint32_t packets_index = 0;
  std::uint8_t *seek = data;
  std::uint16_t file_size = size;
  while (true) {
    std::uint16_t current_sz = 0;
    if (file_size >= 60) {
      std::copy(seek, seek + 60, cmd + 3);
      seek += 60;
      file_size -= 60;
      current_sz = 60;
    } else {
      std::copy(seek, seek + file_size, cmd + 3);
      current_sz = file_size;
      file_size = 0;
    }

    cmd[0] = 0x5A;
    cmd[1] = packets_index;
    cmd[2] = current_sz;
    cmd[current_sz + 3] = check_sum(cmd + 3, current_sz);

    if (hid_->send(0, cmd, 64, 100) <= 0) {
      LOGE("%s %d:: Update file data failure.", __FILE__, __LINE__);
      return false;
    }
    packets_index++;

    if (file_size == 0) {
      cmd[0] = 0xAA;
      cmd[1] = 0xFF;
      if (hid_->send(0, cmd, 64, 100) <= 0) {
        LOGE("%s %d:: Update file data failure.", __FILE__, __LINE__);
        return false;
      } else {
        break;
      }
    }
  }

  return true;
}

bool Channels::SetFiles(device_desc_t *desc,
    imu_params_t *imu_params,
    Version *spec_version) {
  if (desc == nullptr && imu_params == nullptr) {
    LOGE("%s %d:: Files are not provided to set.", __FILE__, __LINE__);
    return false;
  }

  Version *spec_ver = spec_version;
  if (spec_ver == nullptr && desc != nullptr) {
    spec_ver = &desc->spec_version;
  }
  CheckSpecVersion(spec_ver);

  std::uint8_t data[2000]{};

  std::uint16_t size = 3;
  if (desc != nullptr) {
    data[0] |= 0x80 | 0x01 << 0;
    std::uint16_t data_size = to_data(desc, data + size + 3, spec_ver);
    *(data + size) = 0x01 << 0;
    *(data + size + 1) = data_size & 0xFF;
    *(data + size + 2) = (data_size >> 8) & 0xFF;
    size += 3;
    size += data_size;
  }

  data[0] |= 0x80 | 0x00 << 1;

  if (imu_params != nullptr) {
    data[0] |= 0x80 | 0x01 << 2;
    std::uint16_t data_size = to_data(imu_params, data + size + 3, spec_ver);
    *(data + size) = 0x01 << 2;
    *(data + size + 1) = data_size & 0xFF;
    *(data + size + 2) = (data_size >> 8) & 0xFF;
    size += 3;
    size += data_size;
  }

  size -= 3;
  data[1] = static_cast<std::uint8_t>(size & 0xFF);
  data[2] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[size + 3] = check_sum(data + 3, size);
  size += 4;

  if (!PushFileData(data, size)) {
    LOGE("%s %d:: Update file data failure.",
        __FILE__, __LINE__);
    return false;
  }

  return true;
}

bool Channels::IsBetaDevice() const {
  return !is_hid_exist_;
}

bool Channels::HidFirmwareUpdate(const char *filepath) {
  if (!is_hid_exist_) {
    LOGE("\n%s %d:: This device not support update hid firmware.\n",
        __FILE__, __LINE__);
    return false;
  }

  std::uint8_t cmd[64];

#ifdef MYNTEYE_OS_WIN
  LOGE("\n%s %d:: This feature is not supported on Windows."
      "You should use the feature on Ubuntu.\n");
  return false;
  int fd = open(filepath, O_RDONLY | _O_BINARY);
#else
  int fd = open(filepath, O_RDONLY);
#endif
  if (fd < 0) {
    LOGE("\n%s %d:: Opened %s failed.\n", __FILE__, __LINE__, filepath);
    return false;
  }

#ifdef MYNTEYE_OS_WIN
  if (fstat(fd, &stat_) != 0) { return false; }
#else
  if ((fstat(fd, &stat_) != 0) || (!S_ISREG(stat_.st_mode))) { return false; }
#endif
  file_size_ = stat_.st_size;
  packets_sum_ = file_size_ % 60 > 0 ? file_size_ / 60 + 1 : file_size_ / 60;

  cmd[0] = 0xAA;
  cmd[1] = 4;
  cmd[2] = file_size_ & 0xFF;
  cmd[3] = (file_size_ & 0xFF00) >> 8;
  cmd[4] = (file_size_ & 0xFF0000) >> 16;
  cmd[5] = (file_size_ & 0xFF000000) >> 24;

  if (hid_->get_device_class() == 0xFF) {
    LOGI("\nUpdate will start......, "
      "please don't pull out device!\n");
  } else {
    int ret = hid_->send(0, cmd, 64, 10);
    if (ret <= 0) {
      LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
      return false;
    }

    hid_->droped();
    hid_->droped();
    LOGI("\nPlease wait a moment, don't pull out device!\n");

    while (hid_->get_device_class() == -1) {
#ifdef MYNTEYE_OS_LINUX
      sleep(1);
#endif
      int ret = hid_->open(1, -1, -1);
      if (ret > 0) { break; }
      if (++req_count_ > 50) {
        LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
        return false;
      }
    }
    if (hid_->get_device_class() == 0xFF) {
      LOGI("\nUpdate will start......, "
        "please don't pull out device!\n");
    } else {
      LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
      return false;
    }
  }

  if (hid_->send(0, cmd, 64, 10) <= 0) {
    LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
    return false;
  }

#ifdef MYNTEYE_OS_WIN
  if (0x100 == hid_->get_version_number()) {
    cmd[0] = 0x00;
    if (hid_->send(0, cmd, 64, 20000) <= 0) {
      LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
      return false;
    }
    cmd[0] = 0xAB;
  } else {
    if (hid_->receive(0, cmd, 64, 20000) <= 0) {
      LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
      return false;
    }
  }
#else
  if (hid_->receive(0, cmd, 64, 20000) <= 0) {
    LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
    return false;
  }
#endif

  if (0xAB != cmd[0]) {
    LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
    return false;
  }

  while (true) {
    int current_len = read(fd, static_cast<std::uint8_t *>(cmd + 3), 60);
    if (-1 == current_len) {
      LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
      return false;
    }

    cmd[0] = 0x5A;
    cmd[1] = packets_index_;
    cmd[2] = current_len;
    cmd[current_len + 3] = check_sum(
        static_cast<std::uint8_t *>(cmd + 3), current_len);

    if (hid_->send(0, cmd, 64, 100) <= 0) {
      LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
      return false;
    }

    packets_index_++;

    if (current_len < 60) {
      cmd[0] = 0xAA;
      cmd[1] = 0xFF;
      hid_->send(0, cmd, 64, 100);
      break;
    }
  }

  CloseHid();
#ifdef MYNTEYE_OS_LINUX
  sleep(2);
#endif
  OpenHid();
  if (hid_->get_device_class() == 0xFF) {
    LOGI("\nThis upgrade is not valid. Please re-upgrade.\n");
    return false;
  } else {
    LOGI("\nUpdate success.\n");
  }

  LOGI("\nUpdate success.\n");
  return true;
}


MYNTEYE_END_NAMESPACE
