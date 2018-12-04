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
#ifndef MYNTEYE_DEVICE_DEVICE_H_
#define MYNTEYE_DEVICE_DEVICE_H_
#pragma once

#include <condition_variable>
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <map>

#include "eSPDI.h"

#include "mynteyed/stubs/global.h"

#ifdef MYNTEYE_OS_WIN
#include <Windows.h>
#endif

#include "mynteyed/device/device_info.h"
#include "mynteyed/device/open_params.h"
#include "mynteyed/device/stream_info.h"
#include "mynteyed/device/types_internal.h"

#include "mynteyed/device/image.h"

MYNTEYE_BEGIN_NAMESPACE

class Device {
 public:
  using image_size_t = unsigned long int;  // NOLINT

  Device();
  ~Device();

  /** Get all device infos */
  void GetDeviceInfos(std::vector<DeviceInfo>* dev_infos);

  /** Get all stream infos */
  void GetStreamInfos(const std::int32_t& dev_index,
      std::vector<StreamInfo>* color_infos,
      std::vector<StreamInfo>* depth_infos);

  /** Set auto-exposure enabled or not */
  bool SetAutoExposureEnabled(bool enabled);
  /** Set auto-white-balance enabled or not */
  bool SetAutoWhiteBalanceEnabled(bool enabled);

  /** Set infrared interleave */
  void SetInfraredInterleave(const OpenParams& params);

  /** Set infrared intensity */
  void SetInfraredIntensity(std::uint16_t value);

  /** Open device */
  bool Open(const OpenParams& params);

  bool IsOpened() const;
  void CheckOpened(const std::string& event = "") const;
  bool ExpectOpened(const std::string& event) const;

  OpenParams GetOpenParams() const;
  bool IsRightColorSupported() const;
  bool IsRightColorSupported(const StreamMode& stream_mode) const;

  /** Get color image, nullptr if failed */
  Image::pointer GetImageColor();  // cross
  /** Get depth image, nullptr if failed */
  Image::pointer GetImageDepth();  // cross

  /** Get camera calibration. */
  std::shared_ptr<CameraCalibration> GetCameraCalibration(
      const StreamMode& stream_mode);
  /** Get camera calibration file. */
  bool GetCameraCalibrationFile(const StreamMode& stream_mode,
                                const std::string& filename);
  /** Set camera calibration bin file. */
  bool SetCameraCalibrationBinFile(const std::string& filename);

  /** Close device */
  void Close();

 protected:
  /** Get stream index for open */
  void GetStreamIndex(const OpenParams& params,
      int* color_res_index,
      int* depth_res_index);
  /** Get stream index for open */
  void GetStreamIndex(const std::int32_t& dev_index,
      const StreamMode& stream_mode,
      const StreamFormat& color_stream_format,
      const StreamFormat& depth_stream_format,
      int* color_res_index,
      int* depth_res_index);

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

  std::shared_ptr<CameraCalibration> GetCameraCalibration(int index);
  bool GetCameraCalibrationFile(int index, const std::string& filename);

  void SyncCameraCalibrations();

  void CompatibleUSB2(const OpenParams& params);

  void CompatibleMJPG();

 private:
  void Init();
  void OnInit();  // cross

  void ReleaseBuf();

  bool IsUSB2();

#ifdef MYNTEYE_OS_WIN
  static void ImgCallback(EtronDIImageType::Value imgType, int imgId,
      unsigned char* imgBuf, int imgSize, int width, int height,
      int serialNumber, void *pParam);
#endif

  int OpenDevice(const DeviceMode& dev_mode);
  
  void* etron_di_;

  DEVSELINFO dev_sel_info_;
  int depth_data_type_;

  PETRONDI_STREAM_INFO stream_color_info_ptr_;
  PETRONDI_STREAM_INFO stream_depth_info_ptr_;
  int color_res_index_ = 0;
  int depth_res_index_ = 0;
  int framerate_ = 0;

  std::int32_t stream_info_dev_index_ = -1;

  int color_serial_number_ = 0;
  int depth_serial_number_ = 0;
  image_size_t color_image_size_ = 0;
  image_size_t depth_image_size_ = 0;
  Image::pointer color_image_buf_ = nullptr;
  Image::pointer depth_image_buf_ = nullptr;
  unsigned char* depth_buf_ = nullptr;

#ifdef MYNTEYE_OS_WIN
  bool is_color_ok_;
  bool is_depth_ok_;
  std::condition_variable color_condition_;
  std::condition_variable depth_condition_;
  std::mutex color_mtx_;
  std::mutex depth_mtx_;
  RGBQUAD color_palette_z14_[16384];
#else  // MYNTEYE_OS_LINUX
  DEPTH_TRANSFER_CTRL dtc_;
#endif

  DepthMode depth_mode_;

  std::vector<std::shared_ptr<CameraCalibration>> camera_calibrations_;

  OpenParams open_params_;

  bool color_device_opened_;
  bool depth_device_opened_;

  bool ir_interleave_enabled_;
  bool color_interleave_enabled_;
  bool depth_interleave_enabled_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_DEVICE_H_
