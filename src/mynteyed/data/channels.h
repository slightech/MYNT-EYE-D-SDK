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
#ifndef MYNTEYE_DATA_CHANNELS_H_
#define MYNTEYE_DATA_CHANNELS_H_
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <thread>
#include <vector>

#include "mynteyed/data/types_internal.h"
#include "mynteyed/types_data.h"

MYNTEYE_BEGIN_NAMESPACE

namespace hid {

class hid_device;

}  // namespace hid

class MYNTEYE_API Channels {
 public:
  typedef enum FileId {
    FID_DEVICE_DESC = 1,  // device desc
    FID_RESERVE = 2,      // reserve
    FID_IMU_PARAMS = 4,   // imu params
    FID_LAST,
  } file_id_t;

  using device_desc_t = device::Descriptors;
  using imu_params_t = device::ImuParams;

  using imu_packets_t = std::vector<ImuDataPacket>;
  using img_packets_t = std::vector<ImgInfoPacket>;

  using imu_callback_t = std::function<void(const ImuDataPacket &packet)>;
  using img_callback_t = std::function<void(const ImgInfoPacket &packet)>;

  Channels();
  virtual ~Channels();

  bool IsAvaliable() const;
  bool IsOpened() const;

  void SetImuDataCallback(imu_callback_t callback);
  void SetImgInfoCallback(img_callback_t callback);

  bool IsHidAvaliable() const;
  bool IsHidOpened() const;
  bool IsHidTracking() const;

  bool StartHidTracking();
  bool StopHidTracking();

  bool GetFiles(device_desc_t *desc,
      imu_params_t *imu_params,
      Version *spec_version = nullptr);

  bool SetFiles(device_desc_t *desc,
      imu_params_t *imu_params,
      Version *spec_version);

  bool IsBetaDevice() const;

 protected:
  void Detect();
  bool Open();
  void Close();

  void DetectHid();
  bool OpenHid();
  void CloseHid();

 private:
  void DoHidTrack();
  bool DoHidDataExtract(imu_packets_t &imu, img_packets_t &img);  // NOLINT

  bool PullFileData(bool device_info,
      bool reserve,
      bool imu_params,
      std::uint8_t *data,
      std::uint16_t &file_size);  // NOLINT
  bool PushFileData(std::uint8_t *data, std::uint16_t size);

  std::shared_ptr<hid::hid_device> hid_;

  bool is_hid_exist_ = false;
  bool is_hid_opened_ = false;
  bool is_hid_tracking_ = false;

  imu_callback_t imu_callback_;
  img_callback_t img_callback_;

  std::thread hid_track_thread_;

  std::uint16_t package_sn_ = 0;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DATA_CHANNELS_H_
