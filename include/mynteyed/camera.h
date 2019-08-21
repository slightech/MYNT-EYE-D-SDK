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
#include <functional>
#include <limits>
#include <memory>
#include <vector>
#include <string>

#include "mynteyed/device/device_info.h"
#include "mynteyed/device/image.h"
#include "mynteyed/device/open_params.h"
#include "mynteyed/device/stream_info.h"
#include "mynteyed/types.h"

MYNTEYE_BEGIN_NAMESPACE

class CameraPrivate;

class MYNTEYE_API Camera {
 public:
  using img_info_callback_t =
        std::function<void(const std::shared_ptr<ImgInfo>& info)>;
  using stream_callback_t = std::function<void(const StreamData& data)>;
  using motion_callback_t = std::function<void(const MotionData& data)>;
  using location_callback_t = std::function<void(const LocationData& data)>;
  using distance_callback_t = std::function<void(const DistanceData& data)>;

  Camera();
  ~Camera();

  /** Get all device infos */
  std::vector<DeviceInfo> GetDeviceInfos() const;
  /** Get all device infos */
  void GetDeviceInfos(std::vector<DeviceInfo>* dev_infos) const;

  /** Get all stream infos */
  void GetStreamInfos(const std::int32_t& dev_index,
      std::vector<StreamInfo>* color_infos,
      std::vector<StreamInfo>* depth_infos) const;

  /** Open camera */
  ErrorCode Open();
  /** Open camera with params */
  ErrorCode Open(const OpenParams& params);

  /** Whethor camera is opened or not */
  bool IsOpened() const;
  /** Get open params */
  OpenParams GetOpenParams() const;

  /** Get all device descriptors */
  std::shared_ptr<device::Descriptors> GetDescriptors() const;
  /** Get one device descriptor */
  std::string GetDescriptor(const Descriptor &desc) const;

  /** Get the intrinsics of camera */
  StreamIntrinsics GetStreamIntrinsics(const StreamMode& stream_mode) const {
    bool ok = true;
    return GetStreamIntrinsics(stream_mode, &ok);
  }
  /** Get the intrinsics of camera */
  StreamIntrinsics GetStreamIntrinsics(const StreamMode& stream_mode,
      bool* ok) const;
  /** Get the extrinsics of camera */
  StreamExtrinsics GetStreamExtrinsics(const StreamMode& stream_mode) const {
    bool ok = true;
    return GetStreamExtrinsics(stream_mode, &ok);
  }
  /** Get the extrinsics of camera */
  StreamExtrinsics GetStreamExtrinsics(const StreamMode& stream_mode,
      bool* ok) const;

  /** Write camera calibration bin file */
  bool WriteCameraCalibrationBinFile(const std::string& filename);

  /** Get the intrinsics of motion */
  MotionIntrinsics GetMotionIntrinsics() const {
    bool ok = true;
    return GetMotionIntrinsics(&ok);
  }
  /** Get the intrinsics of motion */
  MotionIntrinsics GetMotionIntrinsics(bool* ok) const;
  /** Get the extrinsics from left to motion */
  MotionExtrinsics GetMotionExtrinsics() const {
    bool ok = true;
    return GetMotionExtrinsics(&ok);
  }
  /** Get the extrinsics from left to motion */
  MotionExtrinsics GetMotionExtrinsics(bool* ok) const;

  /** Whethor write device supported or not */
  bool IsWriteDeviceSupported() const;
  /** Write device flash */
  bool WriteDeviceFlash(
      device::Descriptors *desc,
      device::ImuParams *imu_params,
      Version *spec_version = nullptr);

  /** Enable process mode, e.g. imu assembly, temp_drift */
  void EnableProcessMode(const ProcessMode& mode);
  /** Enable process mode, e.g. imu assembly, temp_drift */
  void EnableProcessMode(const std::int32_t& mode);

  /** Whethor image info supported or not */
  bool IsImageInfoSupported() const;
  /**
   * Enable image infos.
   *
   * If sync is false, indicates only can get infos from callback.
   * If sync is true, indicates can get infos from callback or access it from StreamData.
   */
  void EnableImageInfo(bool sync);
  /** Disable image info. */
  void DisableImageInfo();
  /** Whethor image info enabled or not */
  bool IsImageInfoEnabled() const;
  /** Whethor image info synced or not */
  bool IsImageInfoSynced() const;

  /** Whethor stream data of certain image type enabled or not */
  bool IsStreamDataEnabled(const ImageType& type) const;
  /** Has any stream data enabled */
  bool HasStreamDataEnabled() const;

  /** Get latest stream data */
  StreamData GetStreamData(const ImageType& type);
  /** Get cached stream datas */
  std::vector<StreamData> GetStreamDatas(const ImageType& type);

  /** Whethor motion datas supported or not */
  bool IsMotionDatasSupported() const;
  /**
   * Enable motion datas.
   *
   * If max_size <= 0, indicates only can get datas from callback.
   * If max_size > 0, indicates can get datas from callback or using GetMotionDatas().
   *
   * Note: if max_size > 0, the motion datas will be cached until you call GetMotionDatas().
   */
  void EnableMotionDatas(
      std::size_t max_size = std::numeric_limits<std::size_t>::max());
  /** Disable motion datas. */
  void DisableMotionDatas();
  /** Whethor motion datas enabled or not */
  bool IsMotionDatasEnabled() const;

  /** Get cached motion datas. Besides, you can also get them from callback */
  std::vector<MotionData> GetMotionDatas();

  /** Set image info callback. */
  void SetImgInfoCallback(img_info_callback_t callback, bool async = true);

  /** Set stream data callback. */
  void SetStreamCallback(const ImageType& type, stream_callback_t callback,
        bool async = true);

  /** Set motion data callback. */
  void SetMotionCallback(motion_callback_t callback, bool async = true);

  /** Close the camera */
  void Close();

  /** Set exposure time [1ms - 655ms]
   * value -- exposure time value
   * */
  void SetExposureTime(const float &value);
  /** Get exposure time
   * value -- return exposure time value
   * */
  void GetExposureTime(float &value);

  /** Set global gain [1 - 16]
   * value -- global gain value
   * */
  void SetGlobalGain(const float &value);
  /** Get global gain
   * value -- return global gain value
   * */
  void GetGlobalGain(float &value);

  /** set infrared(IR) intensity [0, 10] default 4*/
  void SetIRIntensity(const std::uint16_t &value);

  /** Auto-exposure enabled or not  default enabled*/
  bool AutoExposureControl(bool enable);

  /** Auto-white-balance enabled or not  default enabled*/
  bool AutoWhiteBalanceControl(bool enable);

#ifdef MYNTEYE_DEPRECATED_COMPAT
  /** @deprecated Replaced by OpenParams#device_mode */
  void EnableStreamData(const ImageType& type);
  /** @deprecated Replaced by OpenParams#device_mode */
  void DisableStreamData(const ImageType& type);
#endif

  /** Whethor location datas supported or not */
  bool IsLocationDatasSupported() const;
  /**↩
   * Enable location datas.
   *
   * If max_size <= 0, indicates only can get datas from callback.
   * If max_size > 0, indicates can get datas from callback or using GetLocationDatas().
   *
   * Note: if max_size > 0, the distance datas will be cached until you call GetLocationDatas().
  */
  void EnableLocationDatas(std::size_t max_size = std::numeric_limits<std::size_t>::max());
  /** Disable location datas. */
  void DisableLocationDatas();
  /** Whethor location datas enabled or not */
  bool IsLocationDatasEnabled() const;

  /** Get cached location datas. Besides, you can also get them from callback */
  std::vector<LocationData> GetLocationDatas();

  /** Set location data callback. */
  void SetLocationCallback(location_callback_t callback, bool async = true);

  /** Whethor distance datas supported or not */
  bool IsDistanceDatasSupported() const;
  /**
   * Enable distance datas.
   *
   * If max_size <= 0, indicates only can get datas from callback.
   * If max_size > 0, indicates can get datas from callback or using GetDistanceDatas().
   *
   * Note: if max_size > 0, the distance datas will be cached until you call GetDistanceDatas().
   */
  void EnableDistanceDatas(std::size_t max_size = std::numeric_limits<std::size_t>::max());
  /** Disable distance datas. */
  void DisableDistanceDatas();
  /** Whethor distance datas enabled or not */
  bool IsDistanceDatasEnabled() const;

  /** Get cached distance datas. Besides, you can also get them from callback */
  std::vector<DistanceData> GetDistanceDatas();

  /** Set distance data callback. */
  void SetDistanceCallback(distance_callback_t callback, bool async = true);

  void WaitForStream();

  /** Update auxiliary chip firmware. */
  bool AuxiliaryChipFirmwareUpdate(const char* filepath);

#ifdef MYNTEYE_DEPRECATED_COMPAT
  /** @deprecated Replaced by OpenParams#device_mode */
  bool HidFirmwareUpdate(const char* filepath);
#endif

  /**
   * control status of reconnect
   * default enabled
   * */
  void ControlReconnectStatus(const bool &status);

 private:
  std::unique_ptr<CameraPrivate> p_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_CAMERA_H_
