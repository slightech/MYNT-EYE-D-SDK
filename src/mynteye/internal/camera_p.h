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
class Motions;

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

  /** Get all device infos */
  void GetDeviceInfos(std::vector<DeviceInfo>* dev_infos) const;

  /** Get all stream infos */
  void GetStreamInfos(const std::int32_t& dev_index,
      std::vector<StreamInfo>* color_infos,
      std::vector<StreamInfo>* depth_infos) const;

  /** Open camera */
  ErrorCode Open(const OpenParams& params);

  /** Whethor camera is opened or not */
  bool IsOpened() const;
  /** Check camera is opened, otherwise error */
  void CheckOpened() const;

  /** Get all device descriptors */
  std::shared_ptr<device::Descriptors> GetDescriptors() const;
  /** Get one device descriptor */
  std::string GetDescriptor(const Descriptor &desc) const;

  /** Get the intrinsics of camera */
  StreamIntrinsics GetStreamIntrinsics(const StreamMode& stream_mode);
  /** Get the extrinsics from left to right */
  StreamExtrinsics GetStreamExtrinsics(const StreamMode& stream_mode);

  /** Write camera calibration bin file */
  bool WriteCameraCalibrationBinFile(const std::string& filename);

  /** Get the intrinsics of motion */
  MotionIntrinsics GetMotionIntrinsics() const;
  /** Get the extrinsics from left to motion */
  MotionExtrinsics GetMotionExtrinsics() const;

  /** Write device flash */
  bool WriteDeviceFlash(
      device::Descriptors *desc,
      device::ImuParams *imu_params,
      Version *spec_version = nullptr);

  /** Enable cache motion datas, then get them using GetMotionDatas() */
  void EnableMotionDatas(std::size_t max_size);
  /** Get cached motion datas. Besides, you can also get them from callback */
  std::vector<MotionData> GetMotionDatas();

  /** Close the camera */
  void Close();

  /** @deprecated Get camera calibration */
  CameraCalibration GetCameraCalibration(const StreamMode& stream_mode);
  /** @deprecated Get camera calibration file */
  void GetCameraCalibrationFile(const StreamMode& stream_mode,
                                const std::string& filename);

  // todo

  void EnableImageType(const ImageType& type);

  void EnableImuProcessMode(const ProcessMode &mode);

  /** Get datas of stream and status */
  stream_datas_t RetrieveImage(const ImageType& type, ErrorCode* code);
  /** Get the latest data of stream and status */
  stream_data_t RetrieveLatestImage(const ImageType& type, ErrorCode* code);

  /** Start capture image */
  void StartCaptureImage();
  /** Stop capture image */
  void StopCaptureImage();
  /** Start synthetic image */
  void StartSyntheticImage();
  /** Stop synthetic image */
  void StopSyntheticImage();

 protected:
  std::shared_ptr<Channels> channels() const {
    return channels_;
  }

 private:
  void Init();

  void ReadDeviceFlash();

  /** Set the intrinsics of motion */
  void SetMotionIntrinsics(const MotionIntrinsics &in);
  /** Set the extrinsics from left to motion */
  void SetMotionExtrinsics(const MotionExtrinsics &ex);

  /** Start data tracking */
  bool StartDataTracking();
  /** Stop data tracking */
  void StopDataTracking();

  /** Callback of imu data */
  void ImuDataCallback(const ImuDataPacket &packet);
  /** Callback of image info */
  void ImageInfoCallback(const ImgInfoPacket &packet);

  std::shared_ptr<Device> device_;
  std::shared_ptr<Channels> channels_;
  std::shared_ptr<Motions> motions_;

  std::shared_ptr<device::Descriptors> descriptors_;
  std::shared_ptr<MotionIntrinsics> motion_intrinsics_;
  std::shared_ptr<MotionExtrinsics> motion_extrinsics_;

  std::shared_ptr<StreamIntrinsics> stream_intrinsics_;
  std::shared_ptr<StreamExtrinsics> stream_extrinsics_;

 private:
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

  std::mutex mtx_img_info_;
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

  bool is_start_ = false;

  std::map<ImageType, bool> is_enable_image_;
  StreamMode stream_mode_;

  std::map<ProcessMode, bool> is_process_mode_;
  void TempCompensate(std::shared_ptr<ImuData> data);
  void ScaleAssemCompensate(std::shared_ptr<ImuData> data);
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CAMERA_P_H_
