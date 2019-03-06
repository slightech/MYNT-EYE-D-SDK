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
#include "mynteyed/camera.h"

#include "mynteyed/internal/camera_p.h"
#include "mynteyed/util/log.h"

MYNTEYE_USE_NAMESPACE

Camera::Camera() : p_(new CameraPrivate()) {
  DBG_LOGD(__func__);
}

Camera::~Camera() {
  DBG_LOGD(__func__);
  p_.release();
}

std::vector<DeviceInfo> Camera::GetDeviceInfos() const {
  std::vector<DeviceInfo> device_infos;
  GetDeviceInfos(&device_infos);
  return device_infos;
}

void Camera::GetDeviceInfos(std::vector<DeviceInfo>* dev_infos) const {
  p_->GetDeviceInfos(dev_infos);
}

void Camera::GetStreamInfos(const std::int32_t& dev_index,
    std::vector<StreamInfo>* color_infos,
    std::vector<StreamInfo>* depth_infos) const {
  p_->GetStreamInfos(dev_index, color_infos, depth_infos);
}

ErrorCode Camera::Open() {
  std::vector<DeviceInfo> dev_infos = GetDeviceInfos();
  if (dev_infos.size() <= 0) {
    LOGE("Error: Device not found");
    return ErrorCode::ERROR_CAMERA_OPEN_FAILED;
  }
  return Open(OpenParams(0));
}

ErrorCode Camera::Open(const OpenParams& params) {
  return p_->Open(params);
}

bool Camera::IsOpened() const {
  return p_->IsOpened();
}

std::shared_ptr<device::Descriptors> Camera::GetDescriptors() const {
  return p_->GetDescriptors();
}

std::string Camera::GetDescriptor(const Descriptor &desc) const {
  return p_->GetDescriptor(desc);
}

StreamIntrinsics Camera::GetStreamIntrinsics(
    const StreamMode& stream_mode, bool* ok) const {
  return p_->GetStreamIntrinsics(stream_mode, ok);
}

StreamExtrinsics Camera::GetStreamExtrinsics(
    const StreamMode& stream_mode, bool* ok) const {
  return p_->GetStreamExtrinsics(stream_mode, ok);
}

bool Camera::WriteCameraCalibrationBinFile(const std::string& filename) {
  return p_->WriteCameraCalibrationBinFile(filename);
}

MotionIntrinsics Camera::GetMotionIntrinsics(bool* ok) const {
  return p_->GetMotionIntrinsics(ok);
}

MotionExtrinsics Camera::GetMotionExtrinsics(bool* ok) const {
  return p_->GetMotionExtrinsics(ok);
}

bool Camera::IsWriteDeviceSupported() const {
  return p_->IsWriteDeviceSupported();
}

bool Camera::WriteDeviceFlash(
    device::Descriptors *desc,
    device::ImuParams *imu_params,
    Version *spec_version) {
  return p_->WriteDeviceFlash(desc, imu_params, spec_version);
}

void Camera::EnableProcessMode(const ProcessMode& mode) {
  p_->EnableProcessMode(mode);
}

void Camera::EnableProcessMode(const std::int32_t& mode) {
  p_->EnableProcessMode(mode);
}

bool Camera::IsImageInfoSupported() const {
  return p_->IsImageInfoSupported();
}

void Camera::EnableImageInfo(bool sync) {
  p_->EnableImageInfo(sync);
}

void Camera::DisableImageInfo() {
  p_->DisableImageInfo();
}

bool Camera::IsImageInfoEnabled() const {
  return p_->IsImageInfoEnabled();
}

bool Camera::IsImageInfoSynced() const {
  return p_->IsImageInfoSynced();
}

bool Camera::IsStreamDataEnabled(const ImageType& type) const {
  return p_->IsStreamDataEnabled(type);
}

bool Camera::HasStreamDataEnabled() const {
  return p_->HasStreamDataEnabled();
}

StreamData Camera::GetStreamData(const ImageType& type) {
  return std::move(p_->GetStreamData(type));
}

std::vector<StreamData> Camera::GetStreamDatas(const ImageType& type) {
  return std::move(p_->GetStreamDatas(type));
}

bool Camera::IsMotionDatasSupported() const {
  return p_->IsMotionDatasSupported();
}

void Camera::EnableMotionDatas(std::size_t max_size) {
  p_->EnableMotionDatas(std::move(max_size));
}

void Camera::DisableMotionDatas() {
  p_->DisableMotionDatas();
}

bool Camera::IsMotionDatasEnabled() const {
  return p_->IsMotionDatasEnabled();
}

std::vector<MotionData> Camera::GetMotionDatas() {
  return std::move(p_->GetMotionDatas());
}

void Camera::SetImgInfoCallback(img_info_callback_t callback, bool async) {
  p_->SetImgInfoCallback(callback, async);
}

void Camera::SetStreamCallback(const ImageType& type,
    stream_callback_t callback, bool async) {
  p_->SetStreamCallback(type, callback, async);
}

void Camera::SetMotionCallback(motion_callback_t callback, bool async) {
  p_->SetMotionCallback(callback, async);
}

void Camera::Close() {
  p_->Close();
}

bool Camera::HidFirmwareUpdate(const char* filepath) {
  return p_->HidFirmwareUpdate(filepath);
}

void Camera::SetExposureTime(const float &value) {
  return p_->SetExposureTime(value);
}

void Camera::GetExposureTime(float &value) {
  return p_->GetExposureTime(value);
}

void Camera::SetGlobalGain(const float &value) {
  return p_->SetGlobalGain(value);
}

void Camera::GetGlobalGain(float &value) {
  return p_->GetGlobalGain(value);
}

void Camera::SetIRIntensity(
    const std::uint16_t &value) {
  return p_->SetIRIntensity(value);
}

bool Camera::AutoExposureControl(bool enable) {
  return p_->AutoExposureControl(enable);
}

bool Camera::AutoWhiteBalanceControl(bool enable) {
  return p_->AutoWhiteBalanceControl(enable);
}

void Camera::EnableMatchFrameId() {
  p_->EnableMatchFrameId();
}

#ifdef MYNTEYE_DEPRECATED_COMPAT
void Camera::EnableStreamData(const ImageType& type) {
  LOGW("%s is deprecated, replaced by OpenParams#device_mode.", __func__);
}

void Camera::DisableStreamData(const ImageType& type) {
  LOGW("%s is deprecated, replaced by OpenParams#device_mode.", __func__);
}
#endif
