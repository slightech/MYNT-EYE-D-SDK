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

#include "eSPDI.h"

#include "mynteye/image.h"
#include "mynteye/types.h"
#include "mynteye/internal/types.h"
#include "mynteye/callbacks.h"

MYNTEYE_BEGIN_NAMESPACE

class Rate;
class Channels;

class MYNTEYE_API CameraPrivate {
 public:
  using image_size_t = unsigned long int;  // NOLINT
  using stream_data_t = device::StreamData;
  using stream_datas_t = std::vector<stream_data_t>;
  using motion_data_t = device::MotionData;
  using motion_datas_t = std::vector<motion_data_t>;
  using img_info_data_t = device::ImgInfoData;
  using img_info_datas_t = std::vector<img_info_data_t>;

  CameraPrivate();
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

  bool SetHWPostProcess(bool enable);

  ErrorCode Open(const InitParams& params);

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
  void ImuDataCallback(const ImuPacket &packet);
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

  struct CameraCtrlRectLogData GetHDCameraCtrlData();
  struct CameraCtrlRectLogData GetVGACameraCtrlData();

  void GetCameraLogData(int index);
  struct CameraCtrlRectLogData GetCameraCtrlData(int index);
  void SetCameraLogData(const std::string& file);

  void SyncCameraLogData();

  /** Set image mode (raw image or rectified image) */
  void SetImageMode(const ImageMode &mode);

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

 private:
  void Init();
  void OnInit();
  void OnPreWait();
  void OnPostWait();

  void SyntheticImageColor();
  void SyntheticImageDepth();
  void CaptureImageColor(ErrorCode *code);
  void CaptureImageDepth(ErrorCode *code);

  void TransferColor(Image::pointer color, img_info_data_t info);
  void CutPart(ImageType type, Image::pointer color, img_info_data_t info);

  Image::pointer RetrieveImageColor(ErrorCode* code);
  Image::pointer RetrieveImageDepth(ErrorCode* code);

  void ReleaseBuf();

#ifdef MYNTEYE_OS_WIN
  static void ImgCallback(EtronDIImageType::Value imgType, int imgId,
      unsigned char* imgBuf, int imgSize, int width, int height,
      int serialNumber, void *pParam);
#endif
  std::vector <struct CameraCtrlRectLogData> camera_log_datas_;

  void* etron_di_;

  DEVSELINFO dev_sel_info_;
  int depth_data_type_;

  PETRONDI_STREAM_INFO stream_color_info_ptr_;
  PETRONDI_STREAM_INFO stream_depth_info_ptr_;
  int color_res_index_ = 0;
  int depth_res_index_ = 0;
  int framerate_ = 0;
  std::unique_ptr<Rate> rate_;

  std::int32_t stream_info_dev_index_ = -1;

  int color_serial_number_ = 0;
  int depth_serial_number_ = 0;
  image_size_t color_image_size_ = 0;
  image_size_t depth_image_size_ = 0;
  Image::pointer color_image_buf_ = nullptr;
  Image::pointer depth_image_buf_ = nullptr;
  unsigned char* depth_buf_ = nullptr;

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
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CAMERA_P_H_
