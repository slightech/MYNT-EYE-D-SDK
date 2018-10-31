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

std::vector<DeviceInfo> Camera::GetDevices() const {
  std::vector<DeviceInfo> device_infos;
  GetDevices(&device_infos);
  return device_infos;
}

void Camera::GetDevices(std::vector<DeviceInfo>* dev_infos) const {
  p_->GetDevices(dev_infos);
}

void Camera::GetResolutions(
    const std::int32_t& dev_index,
    std::vector<StreamInfo>* color_infos,
    std::vector<StreamInfo>* depth_infos) const {
  p_->GetResolutions(dev_index, color_infos, depth_infos);
}

ErrorCode Camera::Open() {
  std::vector<DeviceInfo> dev_infos = GetDevices();
  if (dev_infos.size() <= 0) {
    LOGE("Error: Device not found");
    return ErrorCode::ERROR_CAMERA_OPEN_FAILED;
  }
  return Open(InitParams(0));
}

ErrorCode Camera::Open(const InitParams& params) {
  return p_->Open(params);
}

bool Camera::IsOpened() const {
  return p_->IsOpened();
}

void Camera::EnableImageType(const ImageType& type) {
  p_->EnableImageType(type);
}

std::vector<mynteye::StreamData> Camera::RetrieveImages(const ImageType& type) {
  ErrorCode code = ErrorCode::SUCCESS;
  return RetrieveImages(type, &code);
}

std::vector<mynteye::StreamData> Camera::RetrieveImages(
    const ImageType& type, ErrorCode* code) {
  std::vector<mynteye::StreamData> datas;
  for (auto &&data : p_->RetrieveImage(type, code)) {
    mynteye::StreamData tmp = {data.img_info, data.img};
    datas.push_back(tmp);
  }
  return datas;
}

mynteye::StreamData Camera::RetrieveImage(const ImageType& type) {
  ErrorCode code = ErrorCode::SUCCESS;
  return RetrieveImage(type, &code);
}

mynteye::StreamData Camera::RetrieveImage(const ImageType& type,
    ErrorCode* code) {
  auto data = p_->RetrieveLatestImage(type, code);
  return {data.img_info, data.img};
}

std::vector<mynteye::MotionData> Camera::RetrieveMotions() {
  std::vector<mynteye::MotionData> datas;
  for (auto &&data : p_->GetImuDatas()) {
    datas.push_back({data.imu});
  }
  return datas;
}

void Camera::Wait() const {
  p_->Wait();
}

void Camera::Close() {
  p_->Close();
}

void Camera::SetCalibrationWithFile(const std::string &file_name) {
  p_->SetCameraLogData(file_name);
}

void Camera::GetHDCameraLogDataFile() { p_->GetHDCameraLogData(); }
void Camera::GetVGACameraLogDataFile() { p_->GetVGACameraLogData(); }

struct CameraCtrlRectLogData Camera::GetHDCameraCtrlData() {
  return p_->GetHDCameraCtrlData();
}

struct CameraCtrlRectLogData Camera::GetVGACameraCtrlData() {
  return p_->GetVGACameraCtrlData();
}

void Camera::SetImageMode(const ImageMode& mode) {
  p_->SetImageMode(mode);
}

StreamMode Camera::GetStreamMode() {
  return p_->GetStreamMode();
}
