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

MYNTEYE_BEGIN_NAMESPACE

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

  /** Enable image of type */
  void EnableImageType(const ImageType& type);
  /** Get datas of stream */
  std::vector<mynteye::StreamData> RetrieveImages(const ImageType& type);
  /** Get datas of stream and status */
  std::vector<mynteye::StreamData> RetrieveImages(
    const ImageType& type, ErrorCode* code);
  /** Get the latest data of stream. */
  mynteye::StreamData RetrieveImage(const ImageType& type);
  /** Get the latest data of stream and status */
  mynteye::StreamData RetrieveImage(const ImageType& type, ErrorCode* code);

  /** Get Motion Data */
  std::vector<mynteye::MotionData> RetrieveMotions();

  /** Wait according to framerate. */
  void Wait() const;

  /** Close the camera. */
  void Close();
  /** Use the XXX.bin file to calibration the camera.*/
  void SetCalibrationWithFile(const std::string& file_name);
  /** Get camera HD stream config information output to RectfyLog_PUMA_1.txt. */
  void GetHDCameraLogDataFile();
  /** Get camera VGA stream config information output to RectfyLog_PUMA_2.txt. */
  void GetVGACameraLogDataFile();
  /** Get camera stream mode*/
  StreamMode GetStreamMode();

  /** Get camera HD stream config information
    return CameraCtrlRectLogData.*/
  CameraCtrlRectLogData GetHDCameraCtrlData();
  /** Get camera VGA stream config information
    return CameraCtrlRectLogData.*/
  CameraCtrlRectLogData GetVGACameraCtrlData();

  /** Get device information of Info*/
  std::string GetInfo(const Info &info) const;

  /** Get the intrinsics of motion. */
  MotionIntrinsics GetMotionIntrinsics() const;

  /** Get the extrinsics from left to motion. */
  Extrinsics GetMotionExtrinsics() const;

  /** Set imu data process mode. */
  void EnableImuProcessMode(const ProcessMode &mode);

 private:
  std::unique_ptr<CameraPrivate> p_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_CAMERA_H_
