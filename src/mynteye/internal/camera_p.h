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

#include <mutex>
#include <vector>

#include "eSPDI.h"

MYNTEYE_BEGIN_NAMESPACE

class CameraPrivate {
 public:
  using image_size_t = unsigned long int;  // NOLINT

  explicit CameraPrivate(Camera* camera);
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
      const StreamFormat& stream_format,
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

  ErrorCode RetrieveImage(const ImageType& type, cv::Mat* image);

  void Close();

  Camera* camera_;

 private:
  ErrorCode RetrieveImageColor(cv::Mat* color);
  ErrorCode RetrieveImageDepth(cv::Mat* depth);

  void ReleaseBuf();

#ifdef MYNTEYE_OS_WIN
  static void ImgCallback(EtronDIImageType::Value imgType, int imgId,
      unsigned char* imgBuf, int imgSize, int width, int height,
      int serialNumber, void *pParam);

  std::mutex mtx_imgs_;
  bool is_color_rgb24_;
  bool is_color_mjpg_;

  int depth_data_size_;
  RGBQUAD color_palette_z14_[16384];
#endif

  void* etron_di_;

  DEVSELINFO dev_sel_info_;
  int depth_data_type_;

  PETRONDI_STREAM_INFO stream_color_info_ptr_;
  PETRONDI_STREAM_INFO stream_depth_info_ptr_;
  int color_res_index_;
  int depth_res_index_;
#ifndef MYNTEYE_OS_WIN
  DEPTH_TRANSFER_CTRL dtc_;
#endif
  int framerate_;

  std::int32_t stream_info_dev_index_;

  int color_serial_number_;
  int depth_serial_number_;
  image_size_t color_image_size_;
  image_size_t depth_image_size_;
  unsigned char* color_img_buf_;
  unsigned char* color_rgb_buf_;
  unsigned char* depth_img_buf_;
  unsigned char* depth_rgb_buf_;

  DepthMode depth_mode_;
  cv::Mat depth_raw_;
  ushort depth_min_;
  ushort depth_max_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CAMERA_P_H_
