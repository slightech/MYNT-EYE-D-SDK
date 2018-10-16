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
#include <string>

#include "mynteye/device_info.h"
#include "mynteye/image.h"
#include "mynteye/init_params.h"
#include "mynteye/stream_info.h"
#include "mynteye/types.h"
#include "mynteye/callbacks.h"

MYNTEYE_BEGIN_NAMESPACE

struct MYNTEYE_API MotionData {
  /** ImuData */
  std::shared_ptr<ImuData> imu;

  bool operator==(const MotionData &other) const {
    if (imu && other.imu) {
      return imu->flag == other.imu->flag &&
        imu->timestamp == other.imu->timestamp &&
        imu->temperature == other.imu->temperature;
    }
    return false;
  }
};

struct MYNTEYE_API StreamData {
  /** Image information */
  std::shared_ptr<ImgInfo> img_info;

  /** Image data */
  std::shared_ptr<Image> img;

  bool operator==(const StreamData& other) const {
    if (img_info && other.img_info) {
      return img_info->frame_id == other.img_info->frame_id &&
             img_info->timestamp == other.img_info->timestamp &&
             img_info->exposure_time == other.img_info->exposure_time;
    }

    return false;
  }
};

class CameraPrivate;

class MYNTEYE_API Camera {
 public:
  Camera();
  ~Camera();

  /** Get Deveces info */
  std::vector<DeviceInfo> GetDevices() const;
  /** Get Deveces info with parameter.*/
  void GetDevices(std::vector<DeviceInfo>* dev_infos) const;

  /** Get data stream information on chosed device.*/
  void GetResolutions(
      const std::int32_t& dev_index,
      std::vector<StreamInfo>* color_infos,
      std::vector<StreamInfo>* depth_infos) const;

  /** Open camera */
  ErrorCode Open();
  /** Open camera with params */
  ErrorCode Open(const InitParams& params);

  /** Get the work status of the camera true(working)/false(stopped) */
  bool IsOpened() const;

  /** Get the work status of the camera true(working)/false(stopped) */
  std::vector<mynteye::StreamData> RetrieveImages(const ImageType& type);
  std::vector<mynteye::StreamData> RetrieveImages(
    const ImageType& type, ErrorCode* code);
  mynteye::StreamData RetrieveImage(const ImageType& type);
  mynteye::StreamData RetrieveImage(const ImageType& type, ErrorCode* code);

  /** Get Motion Data */
  std::vector<mynteye::MotionData> RetrieveMotions();

  /** Wait according to framerate. */
  void Wait() const;

  /** Close the camera. */
  void Close();
  /** Use the XXX.bin file to calibration the camera.*/
  void SetLogData(const std::string& file_name);
  /** Get camera HD stream config information output to RectfyLog_PUMA_1.txt. */
  void GetHDCameraLogDataFile();
  /** Get camera VGA stream config information output to RectfyLog_PUMA_2.txt. */
  void GetVGACameraLogDataFile();

  /** Get camera HD stream config information 
    return struct CameraCtrlRectLogData.*/
  struct CameraCtrlRectLogData GetHDCameraCtrlData();
  /** Get camera VGA stream config information 
    return struct CameraCtrlRectLogData.*/
  struct CameraCtrlRectLogData GetVGACameraCtrlData();

  void SetImageMode(const ImageMode& mode);

 private:
  std::unique_ptr<CameraPrivate> p_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_CAMERA_H_
