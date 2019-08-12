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
#include "mynteyed/internal/camera_p.h"

#include <stdexcept>
#include <string>
#include <chrono>
#include <utility>

#include "mynteyed/data/channels.h"
#include "mynteyed/device/device.h"
#include "mynteyed/internal/image_utils.h"
#include "mynteyed/internal/motions.h"
#include "mynteyed/internal/location.h"
#include "mynteyed/internal/distance.h"
#include "mynteyed/internal/streams.h"
#include "mynteyed/util/log.h"
#include "mynteyed/util/rate.h"

#define IMG_INFO_ASYNC_MAX_SIZE 120  // 60fps, 2s
#define STREAM_ASYNC_MAX_SIZE 1  // latest
#define MOTION_ASYNC_MAX_SIZE 800  // 400hz, 2s
#define LOCATION_ASYNC_MAX_SIZE 800  // 400hz, 2s
#define DISTANCE_ASYNC_MAX_SIZE 800  // 400hz, 2s

static const int MAX_RELINK_TIMES = 3;

MYNTEYE_USE_NAMESPACE

CameraPrivate::CameraPrivate() : device_(std::make_shared<Device>()) {
  DBG_LOGD(__func__);
  Init();
}

void CameraPrivate::Init() {
  channels_ = std::make_shared<Channels>();
  motions_ = std::make_shared<Motions>();
  location_ = std::make_shared<Location>();
  distance_ = std::make_shared<Distance>();
  streams_ = std::make_shared<Streams>(device_);
  m_filter_manager = std::make_shared<FilterSpigot>();

  reconnect_times_ = 0;

  if (channels_->IsAvaliable()) {
    ReadDeviceFlash();
  }

  enable_reconnect_ = true;
}

CameraPrivate::~CameraPrivate() {
  DBG_LOGD(__func__);
  Close();
  device_ = nullptr;

  if (watch_thread_.joinable()) {
    watch_thread_.join();
  }
}

void CameraPrivate::GetDeviceInfos(std::vector<DeviceInfo>* dev_infos) const {
  return device_->GetDeviceInfos(dev_infos);
}

void CameraPrivate::GetStreamInfos(const std::int32_t& dev_index,
    std::vector<StreamInfo>* color_infos,
    std::vector<StreamInfo>* depth_infos) const {
  return device_->GetStreamInfos(dev_index, color_infos, depth_infos);
}

ErrorCode CameraPrivate::Open(const OpenParams& params) {
  if (IsOpened()) {
    return ErrorCode::SUCCESS;
  }

  bool ok = device_->Open(params);
  if (!ok) {
    return ErrorCode::ERROR_FAILURE;
  }

  if (ok) {
    NotifyDataTrackStateChanged();
    // Enable streams according to device mode
    switch (params.dev_mode) {
      case DeviceMode::DEVICE_COLOR:
        streams_->EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
        if (device_->IsRightColorSupported(params.stream_mode)) {
          streams_->EnableStreamData(ImageType::IMAGE_RIGHT_COLOR);
        }
        break;
      case DeviceMode::DEVICE_DEPTH:
        streams_->EnableStreamData(ImageType::IMAGE_DEPTH);
        break;
      case DeviceMode::DEVICE_ALL:
        streams_->EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
        if (device_->IsRightColorSupported(params.stream_mode)) {
          streams_->EnableStreamData(ImageType::IMAGE_RIGHT_COLOR);
        }
        streams_->EnableStreamData(ImageType::IMAGE_DEPTH);
        break;
    }
    streams_->OnCameraOpen();
#ifdef MYNTEYE_OS_LINUX
    // control whether reconnect
    if (enable_reconnect_) {
      WatchDog();
    }
#endif
    return ErrorCode::SUCCESS;
  } else {
    return ErrorCode::ERROR_CAMERA_OPEN_FAILED;
  }
}

bool CameraPrivate::IsOpened() const {
  return device_->IsOpened();
}

void CameraPrivate::CheckOpened() const {
  device_->CheckOpened();
}

OpenParams CameraPrivate::GetOpenParams() const {
  return device_->GetOpenParams();
}

std::shared_ptr<device::Descriptors> CameraPrivate::GetDescriptors() const {
  return descriptors_;
}

std::string CameraPrivate::GetDescriptor(const Descriptor &desc) const {
  if (!descriptors_ &&
      desc != Descriptor::SERIAL_NUMBER) {
    LOGE("%s %d:: Device information not found", __FILE__, __LINE__);
    return "";
  }
  switch (desc) {
    case Descriptor::DEVICE_NAME:
      return descriptors_->name;
    case Descriptor::SERIAL_NUMBER:
      if (!descriptors_)
        return GetSerialNumber();
      return descriptors_->serial_number;
    case Descriptor::FIRMWARE_VERSION:
      return descriptors_->firmware_version.to_string();
    case Descriptor::HARDWARE_VERSION:
      return descriptors_->hardware_version.to_string();
    case Descriptor::SPEC_VERSION:
      return descriptors_->spec_version.to_string();
    case Descriptor::LENS_TYPE:
      return descriptors_->lens_type.to_string();
    case Descriptor::IMU_TYPE:
      return descriptors_->imu_type.to_string();
    case Descriptor::NOMINAL_BASELINE:
      return std::to_string(descriptors_->nominal_baseline);
    default:
      LOGE("%s %d:: Unknown device info", __FILE__, __LINE__);
      return "";
  }
}

StreamIntrinsics CameraPrivate::GetStreamIntrinsics(
    const StreamMode& stream_mode, bool* ok) {
  StreamIntrinsics in;
  auto calib = GetCameraCalibration(stream_mode);
  if (calib == nullptr || calib->InImgWidth == 0) {
    *ok = false;
    return std::move(in);
  }
  in.left.width = calib->InImgWidth/2;
  in.left.height = calib->InImgHeight;
  in.left.fx = calib->CamMat1[0];
  in.left.fy = calib->CamMat1[4];
  in.left.cx = calib->CamMat1[2];
  in.left.cy = calib->CamMat1[5];
  for (int i = 0; i < 5; i++) {
    in.left.coeffs[i] = calib->CamDist1[i];
  }
  for (int i = 0; i < 12; i++) {
    in.left.p[i] = calib->NewCamMat1[i];
  }
  for (int i = 0; i < 9; i++) {
    in.left.r[i] = calib->LRotaMat[i];
  }
  in.right.width = calib->InImgWidth/2;
  in.right.height = calib->InImgHeight;
  in.right.fx = calib->CamMat2[0];
  in.right.fy = calib->CamMat2[4];
  in.right.cx = calib->CamMat2[2];
  in.right.cy = calib->CamMat2[5];
  for (int i = 0; i < 5; i++) {
    in.right.coeffs[i] = calib->CamDist2[i];
  }
  for (int i = 0; i < 12; i++) {
    in.right.p[i] = calib->NewCamMat2[i];
  }
  for (int i = 0; i < 9; i++) {
    in.right.r[i] = calib->RRotaMat[i];
  }
  *ok = true;
  return std::move(in);
}

StreamExtrinsics CameraPrivate::GetStreamExtrinsics(
    const StreamMode& stream_mode, bool* ok) {
  StreamExtrinsics ex;
  auto calib = GetCameraCalibration(stream_mode);
  if (calib == nullptr || calib->InImgWidth == 0) {
    *ok = false;
    return std::move(ex);
  }
  for (int i = 0; i < 9; i++) {
    ex.rotation[i/3][i%3] = calib->RotaMat[i];
  }
  for (int j = 0; j < 3; j++) {
    ex.translation[j] = calib->TranMat[j];
  }
  *ok = true;
  return std::move(ex);
}

bool CameraPrivate::WriteCameraCalibrationBinFile(const std::string& filename) {
  return device_->SetCameraCalibrationBinFile(filename);
}

MotionIntrinsics CameraPrivate::GetMotionIntrinsics(bool* ok) const {
  if (motion_intrinsics_) {
    *ok = true;
    return *motion_intrinsics_;
  } else {
    *ok = false;
    LOGE("Error: Motion intrinsics not found");
    return {};
  }
}

MotionExtrinsics CameraPrivate::GetMotionExtrinsics(bool* ok) const {
  if (motion_extrinsics_) {
    *ok = true;
    return *motion_extrinsics_;
  } else {
    *ok = false;
    LOGE("Error: Motion extrinsics not found");
    return {};
  }
}

bool CameraPrivate::IsWriteDeviceSupported() const {
  return channels_->IsAvaliable();
}

bool CameraPrivate::WriteDeviceFlash(
    device::Descriptors *desc,
    device::ImuParams *imu_params,
    Version *spec_version) {
  if (!channels_->IsAvaliable()) {
    LOGW("Data channel is unavaliable, could not write device datas.");
    return false;
  }
  return channels_->SetFiles(desc, imu_params, spec_version);
}

void CameraPrivate::EnableProcessMode(const ProcessMode& mode) {
  EnableProcessMode(static_cast<std::int32_t>(mode));
}

void CameraPrivate::EnableProcessMode(const std::int32_t& mode) {
  motions_->EnableProcessMode(mode);
}

bool CameraPrivate::IsImageInfoSupported() const {
  return channels_->IsAvaliable();
}

void CameraPrivate::EnableImageInfo(bool sync) {
  if (!channels_->IsAvaliable()) {
    LOGW("Data channel is unavaliable, could not track image info.");
    return;
  }
  streams_->EnableImageInfo(sync);
  NotifyDataTrackStateChanged();
}

void CameraPrivate::DisableImageInfo() {
  streams_->DisableImageInfo();
  NotifyDataTrackStateChanged();
}

bool CameraPrivate::IsImageInfoEnabled() const {
  return streams_->IsImageInfoEnabled();
}

bool CameraPrivate::IsImageInfoSynced() const {
  return streams_->IsImageInfoSynced();
}

/*
void CameraPrivate::EnableStreamData(const ImageType& type) {
  streams_->EnableStreamData(type);
}

void CameraPrivate::DisableStreamData(const ImageType& type) {
  streams_->DisableStreamData(type);
}
*/

bool CameraPrivate::IsStreamDataEnabled(const ImageType& type) const {
  return streams_->IsStreamDataEnabled(type);
}

bool CameraPrivate::HasStreamDataEnabled() const {
  return streams_->HasStreamDataEnabled();
}

StreamData CameraPrivate::GetStreamData(const ImageType& type) {
  return streams_->GetStreamData(type);
}

std::vector<StreamData> CameraPrivate::GetStreamDatas(const ImageType& type) {
  return streams_->GetStreamDatas(type);
}

bool CameraPrivate::IsExSensorDatasSupported() const {
  return channels_->IsAvaliable();
}

void CameraPrivate::EnableMotionDatas(std::size_t max_size) {
  if (!channels_->IsAvaliable()) {
    LOGW("Data channel is unavaliable, could not track motion datas.");
    return;
  }
  motions_->EnableMotionDatas(std::move(max_size));
  NotifyDataTrackStateChanged();
}

void CameraPrivate::DisableMotionDatas() {
  motions_->DisableMotionDatas();
  NotifyDataTrackStateChanged();
}

bool CameraPrivate::IsMotionDatasEnabled() const {
  return motions_->IsMotionDatasEnabled();
}

std::vector<MotionData> CameraPrivate::GetMotionDatas() {
  return std::move(motions_->GetMotionDatas());
}

void CameraPrivate::SetImgInfoCallback(img_info_callback_t callback,
    bool async) {
  if (async) {
    auto img_info_async_callback =
        AsyncCallback<std::shared_ptr<ImgInfo>>::Create(
            callback, IMG_INFO_ASYNC_MAX_SIZE);
    streams_->SetImgInfoCallback((*img_info_async_callback)());
  } else {
    streams_->SetImgInfoCallback(callback);
  }
}

void CameraPrivate::SetStreamCallback(const ImageType& type,
    stream_callback_t callback, bool async) {
  if (async) {
    auto stream_async_callback =
        AsyncCallback<StreamData>::Create(callback, STREAM_ASYNC_MAX_SIZE);
    streams_->SetStreamCallback(type, (*stream_async_callback)());
  } else {
    streams_->SetStreamCallback(type, callback);
  }
}

void CameraPrivate::SetMotionCallback(motion_callback_t callback, bool async) {
  if (async) {
    auto motion_async_callback =
        AsyncCallback<MotionData>::Create(callback, MOTION_ASYNC_MAX_SIZE);
    motions_->SetMotionCallback((*motion_async_callback)());
  } else {
    motions_->SetMotionCallback(callback);
  }
}

void CameraPrivate::Close() {
  if (!IsOpened()) return;
  StopDataTracking();
  streams_->OnCameraClose();
  device_->Close();
}

std::shared_ptr<CameraCalibration> CameraPrivate::GetCameraCalibration(
    const StreamMode& stream_mode) {
  return device_->GetCameraCalibration(stream_mode);
}

bool CameraPrivate::GetCameraCalibrationFile(
    const StreamMode& stream_mode, const std::string& filename) {
  return device_->GetCameraCalibrationFile(stream_mode, filename);
}

void CameraPrivate::ReadDeviceFlash() {
  if (!channels_->IsAvaliable()) {
    LOGW("Data channel is unavaliable, could not read device datas.");
    return;
  }
  descriptors_ = std::make_shared<device::Descriptors>();

  Channels::imu_params_t imu_params;
  if (!channels_->GetFiles(descriptors_.get(), &imu_params)) {
    LOGE("%s %d:: Read device descriptors failed.", __FILE__, __LINE__);
    return;
  }

  LOGI("\nDevice descriptors:");
  LOGI("  name: %s", descriptors_->name.c_str());
  LOGI("  serial_number: %s", descriptors_->serial_number.c_str());
  LOGI("  firmware_version: %s",
      descriptors_->firmware_version.to_string().c_str());
  LOGI("  hardware_version: %s",
      descriptors_->hardware_version.to_string().c_str());
  LOGI("  spec_version: %s", descriptors_->spec_version.to_string().c_str());
  LOGI("  lens_type: %s", descriptors_->lens_type.to_string().c_str());
  LOGI("  imu_type: %s", descriptors_->imu_type.to_string().c_str());
  LOGI("  nominal_baseline: %u", descriptors_->nominal_baseline);

  if (imu_params.ok) {
    SetMotionIntrinsics({imu_params.in_accel, imu_params.in_gyro});
    SetMotionExtrinsics(imu_params.ex_left_to_imu);
  } else {
    LOGE("%s %d:: Motion intrinsics & extrinsics not exist",
        __FILE__, __LINE__);
  }
}

void CameraPrivate::SetMotionIntrinsics(const MotionIntrinsics &in) {
  if (!motion_intrinsics_) {
    motion_intrinsics_ = std::make_shared<MotionIntrinsics>();
  }
  *motion_intrinsics_ = in;
  motions_->SetMotionIntrinsics(motion_intrinsics_);
}

void CameraPrivate::SetMotionExtrinsics(const MotionExtrinsics &ex) {
  if (!motion_extrinsics_) {
    motion_extrinsics_ = std::make_shared<MotionExtrinsics>();
  }
  *motion_extrinsics_ = ex;
}

bool CameraPrivate::StartDataTracking() {
  if (!channels_->IsHidAvaliable()) {
    // Not tracking if hid is unavaliable
    return false;
  }

  if (!IsOpened()) {
    // ensure start after opened
    return false;
  }
  if (!motions_->IsMotionDatasEnabled() &&
      !streams_->IsImageInfoEnabled() &&
      !location_->IsLocationDatasEnabled() &&
      !distance_->IsDistanceDatasEnabled()) {
    // Not tracking if data & info both disabled
    return false;
  }

  if (motions_->IsMotionDatasEnabled()) {
    channels_->SetImuDataCallback(std::bind(&Motions::OnImuDataCallback,
        motions_, std::placeholders::_1));
  }

  if (location_->IsLocationDatasEnabled()) {
    channels_->SetGPSDataCallback(std::bind(&Location::OnGPSDataCallback,
        location_, std::placeholders::_1));
  }

  if (distance_->IsDistanceDatasEnabled()) {
    channels_->SetDisDataCallback(std::bind(&Distance::OnDisDataCallback,
        distance_, std::placeholders::_1));
  }

  if (streams_->IsImageInfoEnabled()) {
    channels_->SetImgInfoCallback(std::bind(&Streams::OnImageInfoCallback,
          streams_, std::placeholders::_1));
  }

  if (channels_->IsHidTracking()) return true;

  // Start hid tracking will callback imu data & image info
  return channels_->StartHidTracking();
}

void CameraPrivate::StopDataTracking() {
  if (channels_->IsHidTracking()) {
    channels_->StopHidTracking();
  }
}

void CameraPrivate::NotifyDataTrackStateChanged() {
  bool expect_track = motions_->IsMotionDatasEnabled()
      || location_->IsLocationDatasEnabled()
      || distance_->IsDistanceDatasEnabled()
      || streams_->IsImageInfoEnabled();
  if (expect_track) {
    StartDataTracking();
  } else {
    StopDataTracking();
  }
}

void CameraPrivate::SetExposureTime(const float &value) {
  device_->SetExposureTime(value);
}

void CameraPrivate::GetExposureTime(float &value) {
  device_->GetExposureTime(value);
}

void CameraPrivate::SetGlobalGain(const float &value) {
  device_->SetGlobalGain(value);
}

void CameraPrivate::GetGlobalGain(float &value) {
  device_->GetGlobalGain(value);
}

void CameraPrivate::SetIRIntensity(
    const std::uint16_t &value) {
  device_->SetInfraredIntensity(value);
}

bool CameraPrivate::AutoExposureControl(bool enable) {
  return device_->SetAutoExposureEnabled(enable);
}

bool CameraPrivate::AutoWhiteBalanceControl(bool enable) {
  return device_->SetAutoWhiteBalanceEnabled(enable);
}

void CameraPrivate::EnableLocationDatas(std::size_t max_size) {
  if (!channels_->IsAvaliable()) {
    LOGW("Data channel is unavaliable, could not track location datas.");
    return;
  }
  location_->EnableLocationDatas(std::move(max_size));
  NotifyDataTrackStateChanged();
}

void CameraPrivate::DisableLocationDatas() {
  location_->DisableLocationDatas();
  NotifyDataTrackStateChanged();
}

bool CameraPrivate::IsLocationDatasEnabled() const {
  return location_->IsLocationDatasEnabled();
}

std::vector<LocationData> CameraPrivate::GetLocationDatas() {
  return std::move(location_->GetLocationDatas());
}

void CameraPrivate::SetLocationCallback(location_callback_t callback, bool async) {
  if (async) {
    auto location_async_callback =
        AsyncCallback<LocationData>::Create(callback, LOCATION_ASYNC_MAX_SIZE);
    location_->SetLocationCallback((*location_async_callback)());
  } else {
    location_->SetLocationCallback(callback);
  }
}

void CameraPrivate::EnableDistanceDatas(std::size_t max_size) {
  if (!channels_->IsAvaliable()) {
    LOGW("Data channel is unavaliable, could not track distance datas.");
    return;
  }
  distance_->EnableDistanceDatas(std::move(max_size));
  NotifyDataTrackStateChanged();
}

void CameraPrivate::DisableDistanceDatas() {
  distance_->DisableDistanceDatas();
  NotifyDataTrackStateChanged();
}

bool CameraPrivate::IsDistanceDatasEnabled() const {
  return distance_->IsDistanceDatasEnabled();
}

std::vector<DistanceData> CameraPrivate::GetDistanceDatas() {
  return std::move(distance_->GetDistanceDatas());
}

void CameraPrivate::SetDistanceCallback(distance_callback_t callback, bool async) {
  if (async) {
    auto distance_async_callback =
        AsyncCallback<DistanceData>::Create(callback, DISTANCE_ASYNC_MAX_SIZE);
    distance_->SetDistanceCallback((*distance_async_callback)());
  } else {
    distance_->SetDistanceCallback(callback);
  }
}

void CameraPrivate::SetSerialNumber(const std::string &sn) {
  device_->SetSerialNumber(sn);
}

std::string CameraPrivate::GetSerialNumber() const {
  return device_->GetSerialNumber();
}

void CameraPrivate::Reconnect() {
  if (reconnect_times_++ > MAX_RELINK_TIMES)
    throw_error("\n\nThe camera device is disconnected.\n");

  if (channels_->IsAvaliable()) {
    StopDataTracking();
    channels_->CloseHid();
    if (channels_->OpenHid()) {
      NotifyDataTrackStateChanged();
    } else {
      return;
    }
  }

  if (device_->Restart())
    reconnect_times_ = 0;
}

void CameraPrivate::WaitForStream() {
#ifdef MYNTEYE_OS_WIN
  if (!streams_->WaitForStreamData() && enable_reconnect_) {
    // control whether connect
    Reconnect();
  }
#endif
#ifdef MYNTEYE_OS_LINUX
  streams_->WaitForStreamData();
#endif
}

bool CameraPrivate::AuxiliaryChipFirmwareUpdate(const char* filepath) {
  return channels_->HidFirmwareUpdate(filepath);
}

void CameraPrivate::WatchDog() {
  watch_thread_ = std::thread([this](){
    Rate rate(100);
    while (true) {
     if (!device_->UpdateDeviceStatus()) {
       Reconnect();
     }
     rate.Sleep();
    }
  });
}

void CameraPrivate::ControlReconnectStatus(const bool &status) {
  enable_reconnect_ = status;
}
