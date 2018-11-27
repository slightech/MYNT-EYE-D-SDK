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
#include "mynteyed/internal/motions.h"

#include <utility>

#include "mynteyed/util/log.h"

MYNTEYE_USE_NAMESPACE

namespace {

void matrix_3x1(const double (*src1)[3], const double (*src2)[1],
    double (*dst)[1]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 1; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

void matrix_3x3(const double (*src1)[3], const double (*src2)[3],
    double (*dst)[3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

}  // namespace

Motions::Motions()
  : motion_intrinsics_(nullptr),
    proc_mode_(static_cast<const std::int32_t>(ProcessMode::PROC_NONE)),
    is_motion_datas_enabled_(false),
    motion_datas_max_size_(1000),
    motion_callback_(nullptr),
    motion_count_(0) {
}

Motions::~Motions() {
}

void Motions::SetMotionIntrinsics(const std::shared_ptr<MotionIntrinsics>& ex) {
  motion_intrinsics_ = ex;
}

void Motions::EnableProcessMode(const std::int32_t& mode) {
  proc_mode_ = mode;
}

void Motions::EnableMotionDatas(std::size_t max_size) {
  if (is_motion_datas_enabled_ && motion_datas_max_size_ == max_size) {
    return;
  }
  std::lock_guard<std::mutex> _(metux_);
  is_motion_datas_enabled_ = true;
  motion_datas_max_size_ = max_size;
}

void Motions::DisableMotionDatas() {
  if (!is_motion_datas_enabled_) return;
  std::lock_guard<std::mutex> _(metux_);
  is_motion_datas_enabled_ = false;
  motion_datas_max_size_ = 0;
  motion_datas_.clear();
}

bool Motions::IsMotionDatasEnabled() const {
  return is_motion_datas_enabled_;
}

Motions::datas_t Motions::GetMotionDatas() {
  if (!is_motion_datas_enabled_) {
    throw_error("Must enable motion datas before getting them, or you set "
                "motion callback instead");
  }
  std::lock_guard<std::mutex> _(metux_);
  return std::move(motion_datas_);
}

void Motions::SetMotionCallback(motion_callback_t callback) {
  std::lock_guard<std::mutex> _(metux_);
  motion_callback_ = callback;
}

// call in thread of channels
void Motions::OnImuDataCallback(const ImuDataPacket& packet) {
  auto &&imu = std::make_shared<ImuData>();
  imu->flag = packet.flag;
  imu->temperature = static_cast<double>(packet.temperature * 0.125 + 23);
  imu->timestamp = packet.timestamp;

  if (imu->flag == MYNTEYE_IMU_ACCEL) {
    imu->accel[0] = packet.accel_or_gyro[0] * 12.f / 0x10000;
    imu->accel[1] = packet.accel_or_gyro[1] * 12.f / 0x10000;
    imu->accel[2] = packet.accel_or_gyro[2] * 12.f / 0x10000;
    imu->gyro[0] = 0;
    imu->gyro[1] = 0;
    imu->gyro[2] = 0;
  } else if (imu->flag == MYNTEYE_IMU_GYRO) {
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

  if (motion_count_ < 20) {
    ++motion_count_;
    return;
  }

  bool proc_assembly = ((proc_mode_ & ProcessMode::PROC_IMU_ASSEMBLY) > 0);
  bool proc_temp_drift = ((proc_mode_ & ProcessMode::PROC_IMU_TEMP_DRIFT) > 0);
  if (proc_assembly && proc_temp_drift) {
    ProcImuTempDrift(imu);
    ProcImuAssembly(imu);
  } else if (proc_assembly) {
    ProcImuAssembly(imu);
  } else if (proc_temp_drift) {
    ProcImuTempDrift(imu);
  }

  std::lock_guard<std::mutex> _(metux_);

  data_t data = {imu};

  if (motion_datas_max_size_ > 0) {
    // remove the first one if data is full
    if (motion_datas_.size() > motion_datas_max_size_) {
      motion_datas_.erase(motion_datas_.begin());
    }
    motion_datas_.push_back(data);
  }

  // callback
  if (motion_callback_) {
    motion_callback_(data);
  }
}

void Motions::ProcImuAssembly(std::shared_ptr<ImuData> data) const {
  if (nullptr == motion_intrinsics_) return;

  double dst[3][3] = {0};
  if (data->flag == 1) {
    matrix_3x3(motion_intrinsics_->accel.scale,
        motion_intrinsics_->accel.assembly, dst);
    double s[3][1] = {0};
    double d[3][1] = {0};
    for (int i = 0; i < 3; i++) {
      s[i][0] = data->accel[i];
    }
    matrix_3x1(dst, s, d);
    for (int i = 0; i < 3; i++) {
      data->accel[i] = d[i][0];
    }
  } else if (data->flag == 2) {
    matrix_3x3(motion_intrinsics_->gyro.scale,
        motion_intrinsics_->gyro.assembly, dst);
    double s[3][1] = {0};
    double d[3][1] = {0};
    for (int i = 0; i < 3; i++) {
      s[i][0] = data->gyro[i];
    }
    matrix_3x1(dst, s, d);
    for (int i = 0; i < 3; i++) {
      data->gyro[i] = d[i][0];
    }
  }
}

void Motions::ProcImuTempDrift(std::shared_ptr<ImuData> data) const {
  if (nullptr == motion_intrinsics_) return;

  double temp = data->temperature;
  if (data->flag == 1) {
    data->accel[0] -= motion_intrinsics_->accel.x[1] * temp
      + motion_intrinsics_->accel.x[0];
    data->accel[1] -= motion_intrinsics_->accel.y[1] * temp
      + motion_intrinsics_->accel.y[0];
    data->accel[2] -= motion_intrinsics_->accel.z[1] * temp
      + motion_intrinsics_->accel.z[0];
  } else if (data->flag == 2) {
    data->gyro[0] -= motion_intrinsics_->gyro.x[1] * temp
      + motion_intrinsics_->gyro.x[0];
    data->gyro[1] -= motion_intrinsics_->gyro.y[1] * temp
      + motion_intrinsics_->gyro.y[0];
    data->gyro[2] -= motion_intrinsics_->gyro.z[1] * temp
      + motion_intrinsics_->gyro.z[0];
  }
}
