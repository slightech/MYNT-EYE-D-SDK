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
#ifndef MYNTEYE_INTERNAL_CAMERA_P_H_
#define MYNTEYE_INTERNAL_CAMERA_P_H_
#pragma once

#include "mynteye/camera.h"

#ifdef MYNTEYE_OS_WIN
#include <Windows.h>
#endif

#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <condition_variable>
#include <map>

#include "mynteye/data/types_internal.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;
class Channels;

class MYNTEYE_API CameraPrivate {
 public:
  using stream_data_t = StreamData;
  using stream_datas_t = std::vector<stream_data_t>;
  using motion_data_t = MotionData;
  using motion_datas_t = std::vector<motion_data_t>;

  using img_info_data_t = ImgInfoData;
  using img_info_datas_t = std::vector<img_info_data_t>;

  CameraPrivate();
  ~CameraPrivate();

  void GetDevices(std::vector<DeviceInfo>* dev_infos);
  void GetResolutions(const std::int32_t& dev_index,
      std::vector<StreamInfo>* color_infos,
      std::vector<StreamInfo>* depth_infos);

  ErrorCode SetAutoExposureEnabled(bool enabled);
  ErrorCode SetAutoWhiteBalanceEnabled(bool enabled);

  ErrorCode Open(const OpenParams& params);

  bool IsOpened() const;
  void CheckOpened() const;

  /** Get datas of stream and status */
  stream_datas_t RetrieveImage(const ImageType& type, ErrorCode* code);
  /** Get the latest data of stream and status */
  stream_data_t RetrieveLatestImage(const ImageType& type, ErrorCode* code);

  /** Start hid device */
  bool StartHidTracking();
  // void StopHidTracking();
  /** Set callback of hid */
  void SetHidCallback();
  /** Callback of imu data */
  void ImuDataCallback(const ImuDataPacket &packet);
  /** Callback of image information */
  void ImageInfoCallback(const ImgInfoPacket &packet);
  /** Start capture image */
  void StartCaptureImage();
  /** Stop capture image */
  void StopCaptureImage();
  /** Start synthetic image */
  void StartSyntheticImage();
  /** Stop synthetic image */
  void StopSyntheticImage();
  /** Get imu data */
  motion_datas_t GetImuDatas();

  void EnableImageType(const ImageType& type);

  /** Wait according to framerate. */
  void Wait();
  void Close();

  void GetHDCameraLogData();
  void GetVGACameraLogData();

  CameraCtrlRectLogData GetHDCameraCtrlData();
  CameraCtrlRectLogData GetVGACameraCtrlData();

  void SetCameraLogData(const std::string& file);

  /** Get the device info. */
  std::shared_ptr<DeviceParams> GetInfo() const;
  /** Get the device info of a field. */
  std::string GetInfo(const Info &info) const;
  /** Get the intrinsics of motion. */
  MotionIntrinsics GetMotionIntrinsics() const;
  /** Get the extrinsics from left to motion. */
  Extrinsics GetMotionExtrinsics() const;
  /** Set the intrinsics of motion. */
  void SetMotionIntrinsics(const MotionIntrinsics &in);
  /** Set the extrinsics from left to motion. */
  void SetMotionExtrinsics(const Extrinsics &ex);

  // protected:
  std::shared_ptr<Channels> channels() const {
    return channels_;
  }

  StreamMode GetStreamMode() { return stream_mode_; }

  void EnableImuProcessMode(const ProcessMode &mode);

 protected:
  void IsHidExist();

 private:
  void Init();

  void SyntheticImageColor();
  void SyntheticImageDepth();
  void OldSyntheticImageColor();
  void CaptureImageColor(ErrorCode *code);
  void CaptureImageDepth(ErrorCode *code);

  void TransferColor(Image::pointer color, img_info_data_t info);
  void CutPart(ImageType type, Image::pointer color, img_info_data_t info);
  void OldTransferColor(Image::pointer color);
  void OldCutPart(ImageType type, Image::pointer color);

  Image::pointer RetrieveImageColor(ErrorCode* code);
  Image::pointer RetrieveImageDepth(ErrorCode* code);

  std::shared_ptr<Device> device_;

  std::shared_ptr<Channels> channels_;
  std::mutex mtx_img_info_;
  std::mutex mtx_imu_;

  motion_datas_t imu_data_;
  motion_datas_t cache_imu_data_;
  img_info_datas_t img_info_;
  img_info_datas_t cache_image_info_;

  std::mutex cap_color_mtx_;
  std::mutex cap_depth_mtx_;

  std::thread cap_image_thread_;
  std::thread sync_thread_;

  std::condition_variable image_color_wait_;
  std::condition_variable image_depth_wait_;

  std::vector<Image::pointer> image_color_;
  std::vector<Image::pointer> image_depth_;
  stream_datas_t color_data_;
  stream_datas_t left_color_data_;
  stream_datas_t right_color_data_;
  stream_datas_t depth_data_;
  bool is_capture_image_ = false;
  bool is_synthetic_image_ = false;
  bool is_imu_open_ = false;

  bool is_start_ = false;

  std::map<ImageType, bool> is_enable_image_;
  StreamMode stream_mode_;

  void ReadAllInfos();
  std::shared_ptr<DeviceParams> device_params_;

  std::shared_ptr<MotionIntrinsics> motion_intrinsics_;
  std::shared_ptr<Extrinsics> motion_from_extrinsics_;
  std::size_t motion_count_ = 0;

  std::map<ProcessMode, bool> is_process_mode_;
  void TempCompensate(std::shared_ptr<ImuData> data);
  void ScaleAssemCompensate(std::shared_ptr<ImuData> data);

  bool is_hid_exist_ = false;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CAMERA_P_H_
