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
#include "mynteye/camera.h"

#include "mynteye/internal/camera_p.h"
#include "mynteye/util/log.h"

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

std::shared_ptr<DeviceParams> Camera::GetInfo() const {
  return p_->GetInfo();
}

std::string Camera::GetInfo(const Info &info) const {
  return p_->GetInfo(info);
}

CameraCalibration Camera::GetCameraCalibration(
    const StreamMode& stream_mode) {
  return p_->GetCameraCalibration(stream_mode);
}

void Camera::GetCameraCalibrationFile(
    const StreamMode& stream_mode, const std::string& filename) {
  p_->GetCameraCalibrationFile(stream_mode, filename);
}

void Camera::WriteCameraCalibrationBinFile(const std::string& filename) {
  p_->WriteCameraCalibrationBinFile(filename);
}

MotionIntrinsics Camera::GetMotionIntrinsics() const {
  return p_->GetMotionIntrinsics();
}

Extrinsics Camera::GetMotionExtrinsics() const {
  return p_->GetMotionExtrinsics();
}

void Camera::Wait() const {
}

void Camera::Close() {
  p_->Close();
}

// todo

void Camera::EnableImageType(const ImageType& type) {
  p_->EnableImageType(type);
}

std::vector<mynteye::StreamData> Camera::RetrieveImages(const ImageType& type) {
  ErrorCode code = ErrorCode::SUCCESS;
  return RetrieveImages(type, &code);
}

std::vector<mynteye::StreamData> Camera::RetrieveImages(
    const ImageType& type, ErrorCode* code) {
  return p_->RetrieveImage(type, code);
}

mynteye::StreamData Camera::RetrieveImage(const ImageType& type) {
  ErrorCode code = ErrorCode::SUCCESS;
  return RetrieveImage(type, &code);
}

mynteye::StreamData Camera::RetrieveImage(const ImageType& type,
    ErrorCode* code) {
  return p_->RetrieveLatestImage(type, code);
}

std::vector<mynteye::MotionData> Camera::RetrieveMotions() {
  std::vector<mynteye::MotionData> datas;
  for (auto &&data : p_->GetImuDatas()) {
    datas.push_back({data.imu});
  }
  return datas;
}

void Camera::EnableImuProcessMode(const ProcessMode &mode) {
  return p_->EnableImuProcessMode(mode);
}

// @Deprecated

std::vector<DeviceInfo> Camera::GetDevices() const {
  return GetDeviceInfos();
}

void Camera::GetDevices(std::vector<DeviceInfo>* dev_infos) const {
  GetDeviceInfos(dev_infos);
}

void Camera::GetResolutions(
    const std::int32_t& dev_index,
    std::vector<StreamInfo>* color_infos,
    std::vector<StreamInfo>* depth_infos) const {
  GetStreamInfos(dev_index, color_infos, depth_infos);
}

CameraCtrlRectLogData Camera::GetHDCameraCtrlData() {
  return GetCameraCalibration(StreamMode::STREAM_1280x720);
}

CameraCtrlRectLogData Camera::GetVGACameraCtrlData() {
  return GetCameraCalibration(StreamMode::STREAM_640x480);
}

void Camera::GetHDCameraLogDataFile() {
  GetCameraCalibrationFile(StreamMode::STREAM_1280x720,
      "RectfyLog_PUMA_1.txt");
}

void Camera::GetVGACameraLogDataFile() {
  GetCameraCalibrationFile(StreamMode::STREAM_640x480,
      "RectfyLog_PUMA_2.txt");
}

void Camera::SetCalibrationWithFile(const std::string &file_name) {
  WriteCameraCalibrationBinFile(file_name);
}
