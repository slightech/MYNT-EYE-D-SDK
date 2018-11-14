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
#include "mynteye/internal/motions.h"

#include <utility>

#include "mynteye/util/log.h"

MYNTEYE_USE_NAMESPACE

Motions::Motions()
  : is_motion_datas_enabled_(false),
    motion_datas_max_size_(1000) {
}

Motions::~Motions() {
}

void Motions::EnableMotionDatas(std::size_t max_size) {
  if (max_size <= 0) {
    throw_error("Could not enable motion datas with max_size <= 0");
  }
  is_motion_datas_enabled_ = true;
  motion_datas_max_size_ = max_size;
}

bool Motions::IsMotionDatasEnabled() {
  return is_motion_datas_enabled_;
}

Motions::datas_t Motions::GetMotionDatas() {
  if (!is_motion_datas_enabled_) {
    throw_error("Must enable motion datas before getting them, or you set "
                "motion callback instead");
  }
  std::lock_guard<std::mutex> _(mtx_datas_);
  return std::move(motion_datas_);
}

void Motions::ImuDataCallback(const ImuDataPacket &packet) {
  auto &&imu = std::make_shared<ImuData>();
  imu->flag = packet.flag;
  imu->temperature = static_cast<double>(packet.temperature * 0.125 + 23);
  imu->timestamp = packet.timestamp;

  if (imu->flag == 1) {
    imu->accel[0] = packet.accel_or_gyro[0] * 12.f / 0x10000;
    imu->accel[1] = packet.accel_or_gyro[1] * 12.f / 0x10000;
    imu->accel[2] = packet.accel_or_gyro[2] * 12.f / 0x10000;
    imu->gyro[0] = 0;
    imu->gyro[1] = 0;
    imu->gyro[2] = 0;
  } else if (imu->flag == 2) {
    imu->accel[0] = 0;
    imu->accel[1] = 0;
    imu->accel[2] = 0;
    imu->gyro[0] = packet.accel_or_gyro[0] * 2000.f / 0x10000;
    imu->gyro[1] = packet.accel_or_gyro[1] * 2000.f / 0x10000;
    imu->gyro[2] = packet.accel_or_gyro[2] * 2000.f / 0x10000;
  } else {
    LOGW("Unaccpected imu, flag=%d is wrong", imu->flag);
    return;
  }

  // if (is_process_mode_[ProcessMode::ASSEMBLY]) {
  //   ScaleAssemCompensate(imu);
  // } else if (is_process_mode_[ProcessMode::WARM_DRIFT]) {
  //   TempCompensate(imu);
  // } else if (is_process_mode_[ProcessMode::ALL]) {
  //   TempCompensate(imu);
  //   ScaleAssemCompensate(imu);
  // }

  std::lock_guard<std::mutex> _(mtx_datas_);

  if (motion_datas_.size() > motion_datas_max_size_) {
    motion_datas_.erase(motion_datas_.begin());
  }

  data_t data = {imu};
  if (is_motion_datas_enabled_) {
    motion_datas_.push_back(data);
  }
}
