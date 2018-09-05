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
#include "camera.h"
#include "camera_p.h"

#include "log.hpp"

using namespace mynteye;

Camera::Camera() : d_ptr(new CameraPrivate(this)) {
    DBG_LOGD(__func__);
}

Camera::~Camera() {
    DBG_LOGD(__func__);
}

std::vector<DeviceInfo> Camera::GetDevices() const {
    std::vector<DeviceInfo> device_infos;
    GetDevices(device_infos);
    return device_infos;
}

void Camera::GetDevices(std::vector<DeviceInfo> &dev_infos) const {
    d_ptr->GetDevices(dev_infos);
}

void Camera::GetResolutions(const std::int32_t &dev_index,
        std::vector<StreamInfo> &color_infos,
        std::vector<StreamInfo> &depth_infos) const {
    d_ptr->GetResolutions(dev_index, color_infos, depth_infos);
}

ErrorCode Camera::Open() {
    std::vector<DeviceInfo> dev_infos = GetDevices();
    if (dev_infos.size() <= 0) {
        LOGE("Error: Device not found");
        return ErrorCode::ERROR_CAMERA_OPEN_FAILED;
    }
    return Open(InitParams(0));
}

ErrorCode Camera::Open(const InitParams &params) {
    return d_ptr->Open(params);
}

bool Camera::IsOpened() {
    return d_ptr->IsOpened();
}

ErrorCode Camera::RetrieveImage(cv::Mat &color, cv::Mat &depth) {
    return d_ptr->RetrieveImage(color, depth);
}

/*
ErrorCode Camera::RetrieveImage(cv::Mat &mat, const View &view) {
    return d_ptr->RetrieveImage(mat, view);
}
*/

void Camera::Close() {
    d_ptr->Close();
}
