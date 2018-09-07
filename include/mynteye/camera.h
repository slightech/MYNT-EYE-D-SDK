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
#ifndef MYNTEYE_CAMERA_H_
#define MYNTEYE_CAMERA_H_
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <opencv2/core/core.hpp>

#include "mynteye/device_info.h"
#include "mynteye/init_params.h"
#include "mynteye/stream_info.h"

MYNTEYE_BEGIN_NAMESPACE

class CameraPrivate;

class MYNTEYE_API Camera {
 public:
  Camera();
  ~Camera();

  std::vector<DeviceInfo> GetDevices() const;
  void GetDevices(std::vector<DeviceInfo>* dev_infos) const;

  void GetResolutions(
      const std::int32_t& dev_index,
      std::vector<StreamInfo>* color_infos,
      std::vector<StreamInfo>* depth_infos) const;

  ErrorCode Open();
  ErrorCode Open(const InitParams& params);

  bool IsOpened() const;

  ErrorCode RetrieveImage(const ImageType& type, cv::Mat* image);

  void Close();

 private:
  std::unique_ptr<CameraPrivate> p_;

  friend class CameraPrivate;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_CAMERA_H_
