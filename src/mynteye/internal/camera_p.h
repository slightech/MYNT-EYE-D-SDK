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

#include <memory>
#include <mutex>
#include <vector>
#include <thread>

#include "eSPDI.h"

#include "mynteye/image.h"
#include "mynteye/types.h"
#include "types.h"

MYNTEYE_BEGIN_NAMESPACE

class Rate;
class Channels;

class CameraPrivate {
 public:
  using image_size_t = unsigned long int;  // NOLINT
  using stream_datas_t = std::vector<device::StreamData>; 

  explicit CameraPrivate();
  ~CameraPrivate();

  void GetDevices(std::vector<DeviceInfo>* dev_infos);
  void GetResolutions(const std::int32_t& dev_index,
      std::vector<StreamInfo>* color_infos,
      std::vector<StreamInfo>* depth_infos);

  void GetResolutionIndex(const InitParams& params,
      int* color_res_index,
      int* depth_res_index);
  void GetResolutionIndex(const std::int32_t& dev_index,
      const StreamMode& stream_mode,
      const StreamFormat& color_stream_format,
      const StreamFormat& depth_stream_format,
      int* color_res_index,
      int* depth_res_index);

  ErrorCode SetAutoExposureEnabled(bool enabled);
  ErrorCode SetAutoWhiteBalanceEnabled(bool enabled);

  bool GetSensorRegister(int id, std::uint16_t address, std::uint16_t* value,
      int flag = FG_Address_1Byte);
  bool GetHWRegister(std::uint16_t address, std::uint16_t* value,
      int flag = FG_Address_1Byte);
  bool GetFWRegister(std::uint16_t address, std::uint16_t* value,
      int flag = FG_Address_1Byte);

  bool SetSensorRegister(int id, std::uint16_t address, std::uint16_t value,
      int flag = FG_Address_1Byte);
  bool SetHWRegister(std::uint16_t address, std::uint16_t value,
      int flag = FG_Address_1Byte);
  bool SetFWRegister(std::uint16_t address, std::uint16_t value,
      int flag = FG_Address_1Byte);

  ErrorCode Open(const InitParams& params);

  bool IsOpened() const;
  void CheckOpened() const;

  stream_datas_t RetrieveImage(const ImageType& type, ErrorCode* code);

  void StartHidTracking();
  void StopHidTracking();
  void SetHidCallback();
  void CallbackImuData(const ImuPacket &packet);
  void CallbackImgInfoData(const ImgPacket &packet);
  /** Get imu data */
  std::vector<device::MotionData> GetImuData();

  /** Wait according to framerate. */
  void Wait();
  void Close();

 private:
  void OnInit();
  void OnPreWait();
  void OnPostWait();

  Image::pointer RetrieveImageColor(ErrorCode* code);
  Image::pointer RetrieveImageDepth(ErrorCode* code);

  stream_datas_t Synchronization(const ImageType &type, ErrorCode *code);
  std::mutex cap_color_mutex_;
  std::mutex cap_depth_mutex_;
  std::thread capture_thread_;
  std::vector<Image> image_color_;
  std::vector<Image> image_depth_;
  bool is_capture_image_;

  void ReleaseBuf();

#ifdef MYNTEYE_OS_WIN
  static void ImgCallback(EtronDIImageType::Value imgType, int imgId,
      unsigned char* imgBuf, int imgSize, int width, int height,
      int serialNumber, void *pParam);
#endif

  void* etron_di_;

  DEVSELINFO dev_sel_info_;
  int depth_data_type_;

  PETRONDI_STREAM_INFO stream_color_info_ptr_;
  PETRONDI_STREAM_INFO stream_depth_info_ptr_;
  int color_res_index_;
  int depth_res_index_;
  int framerate_;
  std::unique_ptr<Rate> rate_;

  std::int32_t stream_info_dev_index_;

  int color_serial_number_;
  int depth_serial_number_;
  image_size_t color_image_size_;
  image_size_t depth_image_size_;
  Image::pointer color_image_buf_;
  Image::pointer depth_image_buf_;
  unsigned char* depth_buf_;

#ifdef MYNTEYE_OS_WIN
  std::mutex mtx_imgs_;
  RGBQUAD color_palette_z14_[16384];
#else  // MYNTEYE_OS_LINUX
  DEPTH_TRANSFER_CTRL dtc_;
#endif

  DepthMode depth_mode_;

  std::shared_ptr<Channels> channels_;
  std::mutex mtx_img_info_;
  std::mutex mtx_imu_;

  std::vector<device::MotionData> imu_data_;
  std::vector<device::StreamData> stream_data_;
  std::vector<device::ImgInfoData> img_info_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CAMERA_P_H_
