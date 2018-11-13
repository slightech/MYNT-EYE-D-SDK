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
#include <string.h>
#include <fstream>

#include <stdexcept>
#include <string>
#include <chrono>

#include "mynteye/data/channels.h"
#include "mynteye/device/device.h"
#include "mynteye/internal/camera_p.h"
#include "mynteye/util/log.h"
#include "mynteye/util/rate.h"
#include "mynteye/util/times.h"

MYNTEYE_USE_NAMESPACE

namespace {

void matrix_3x1(const double (*src1)[3], const double (*src2)[1],
    double (*dst)[1]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 1; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

void matrix_3x3(const double (*src1)[3], const double (*src2)[3],
    double (*dst)[3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

}  // namespace

CameraPrivate::CameraPrivate() : device_(std::make_shared<Device>()) {
  DBG_LOGD(__func__);

  Init();
  if (is_hid_exist_) {
    ReadAllInfos();
  }
}

void CameraPrivate::Init() {
  is_enable_image_ = {{ImageType::IMAGE_LEFT_COLOR, false},
                      {ImageType::IMAGE_RIGHT_COLOR, false},
                      {ImageType::IMAGE_DEPTH, false}};

  is_process_mode_ = {{ProcessMode::ASSEMBLY, false},
                      {ProcessMode::WARM_DRIFT, false},
                      {ProcessMode::ALL, false}};

  channels_ = std::make_shared<Channels>();
  IsHidExist();
}

CameraPrivate::~CameraPrivate() {
  DBG_LOGD(__func__);
  if (is_hid_exist_) {
    channels_->StopHidTracking();
  }
  if (is_capture_image_) {
    StopCaptureImage();
  }
  device_ = nullptr;
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
  bool ok = device_->Open(params);
  if (!ok) {
    return ErrorCode::ERROR_FAILURE;
  }

  if (ok) {
    if (is_hid_exist_) {
      if (!StartHidTracking()) {
        return ErrorCode::ERROR_IMU_OPEN_FAILED;
      }
    }
    StartCaptureImage();
    StartSyntheticImage();
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

std::shared_ptr<DeviceParams> CameraPrivate::GetInfo() const {
  return device_params_;
}

std::string CameraPrivate::GetInfo(const Info &info) const {
  if (!device_params_) {
    LOGE("%s %d:: Device information not found", __FILE__, __LINE__);
    return "";
  }
  switch (info) {
    case Info::DEVICE_NAME:
      return device_params_->name;
    case Info::SERIAL_NUMBER:
      return device_params_->serial_number;
    case Info::FIRMWARE_VERSION:
      return device_params_->firmware_version.to_string();
    case Info::HARDWARE_VERSION:
      return device_params_->hardware_version.to_string();
    case Info::SPEC_VERSION:
      return device_params_->spec_version.to_string();
    case Info::LENS_TYPE:
      return device_params_->lens_type.to_string();
    case Info::IMU_TYPE:
      return device_params_->imu_type.to_string();
    case Info::NOMINAL_BASELINE:
      return std::to_string(device_params_->nominal_baseline);
    default:
      LOGE("%s %d:: Unknown device info", __FILE__, __LINE__);
      return "";
  }
}

CameraCalibration CameraPrivate::GetCameraCalibration(
    const StreamMode& stream_mode) {
  return device_->GetCameraCalibration(stream_mode);
}

void CameraPrivate::GetCameraCalibrationFile(
    const StreamMode& stream_mode, const std::string& filename) {
  return device_->GetCameraCalibrationFile(stream_mode, filename);
}

void CameraPrivate::WriteCameraCalibrationBinFile(const std::string& filename) {
  device_->SetCameraCalibrationBinFile(filename);
}

MotionIntrinsics CameraPrivate::GetMotionIntrinsics() const {
  if (motion_intrinsics_) {
    return *motion_intrinsics_;
  } else {
    LOGE("Error: Motion intrinsics not found");
    return {};
  }
}

Extrinsics CameraPrivate::GetMotionExtrinsics() const {
  if (motion_from_extrinsics_) {
    return *motion_from_extrinsics_;
  } else {
    LOGE("Error: Motion extrinsics not found");
    return {};
  }
}

void CameraPrivate::Close() {
  if (device_->IsOpened()) {
    StopCaptureImage();
    StopSyntheticImage();
    channels_->StopHidTracking();
  }
  device_->Close();
}

// ...

void CameraPrivate::SetMotionIntrinsics(const MotionIntrinsics &in) {
  if (!motion_intrinsics_) {
    motion_intrinsics_ = std::make_shared<MotionIntrinsics>();
  }
  *motion_intrinsics_ = in;
}

void CameraPrivate::SetMotionExtrinsics(const Extrinsics &ex) {
  if (!motion_from_extrinsics_) {
    motion_from_extrinsics_ = std::make_shared<Extrinsics>();
  }
  *motion_from_extrinsics_ = ex;
}

// todo

std::vector<StreamData> CameraPrivate::RetrieveImage(const ImageType& type,
    ErrorCode* code) {
  if (!IsOpened()) {
    *code = ErrorCode::ERROR_CAMERA_NOT_OPENED;
    return {};
  }

  switch (type) {
    case ImageType::IMAGE_LEFT_COLOR: {
      std::lock_guard<std::mutex> _(cap_color_mtx_);
      stream_datas_t data = left_color_data_;
      left_color_data_.clear();
      return data;
    } break;
    case ImageType::IMAGE_RIGHT_COLOR: {
      if (!is_enable_image_[ImageType::IMAGE_RIGHT_COLOR]) {
        LOGE("RetrieveImage: Right color is disable.");
        throw new std::runtime_error("RetrieveImage: Right color is disable.");
      }
      std::lock_guard<std::mutex> _(cap_color_mtx_);
      stream_datas_t data = right_color_data_;
      right_color_data_.clear();
      return data;
    } break;
    case ImageType::IMAGE_DEPTH: {
      std::lock_guard<std::mutex> _(cap_depth_mtx_);
      stream_datas_t data = depth_data_;
      depth_data_.clear();
      return data;
    } break;
    default:
      throw new std::runtime_error("RetrieveImage: ImageType is unknown");
  }
}

CameraPrivate::stream_data_t CameraPrivate::RetrieveLatestImage(const ImageType& type,
    ErrorCode* code) {
  if (!IsOpened()) {
    *code = ErrorCode::ERROR_CAMERA_NOT_OPENED;
    return {};
  }

  switch (type) {
    case ImageType::IMAGE_LEFT_COLOR: {
      std::lock_guard<std::mutex> _(cap_color_mtx_);
      if (left_color_data_.empty()) { return {}; }
      auto data = left_color_data_.back();
      left_color_data_.clear();
      return data;
    } break;
    case ImageType::IMAGE_RIGHT_COLOR: {
      std::lock_guard<std::mutex> _(cap_color_mtx_);
      if (right_color_data_.empty()) { return {}; }
      auto data = right_color_data_.back();
      right_color_data_.clear();
      return data;
    } break;
    case ImageType::IMAGE_DEPTH: {
      std::lock_guard<std::mutex> _(cap_depth_mtx_);
      if (depth_data_.empty()) { return {}; }
      auto data = depth_data_.back();
      depth_data_.clear();
      return data;
    } break;
    default:
      throw new std::runtime_error("RetrieveImage: ImageType is unknown");
  }
}

void CameraPrivate::CaptureImageColor(ErrorCode* code) {
  std::unique_lock<std::mutex> _(cap_color_mtx_);
  auto p = RetrieveImageColor(code);
  if (p) {
    auto color = p->Clone();
    image_color_.push_back(color);
    image_color_wait_.notify_one();
  }
}

void CameraPrivate::CaptureImageDepth(ErrorCode* code) {
  std::unique_lock<std::mutex> _(cap_depth_mtx_);
  auto p = RetrieveImageDepth(code);
  if (p) {
    auto depth = p->Clone();
    image_depth_.push_back(depth);
    image_depth_wait_.notify_one();
  }
}

void CameraPrivate::SyntheticImageColor() {
  std::unique_lock<std::mutex> _(cap_color_mtx_);
  image_color_wait_.wait_for(_, std::chrono::seconds(1));

  if (image_color_.empty() || img_info_.empty()) { return; }

  if (image_color_.front()->frame_id() >
      img_info_.back().img_info->frame_id) {
    if (image_color_.size() > 5) { image_color_.clear(); }
    img_info_.clear();
    return;
  } else if (image_color_.back()->frame_id() <
      img_info_.front().img_info->frame_id) {
    if (img_info_.size() > 5) { img_info_.clear(); }
    image_color_.clear();
    return;
  }

  for (auto color : image_color_) {
    for (auto info : img_info_) {
      if (color->frame_id() == info.img_info->frame_id) {
        TransferColor(color, info);
        if (left_color_data_.size() > 30) { left_color_data_.clear(); }
        if (right_color_data_.size() > 30) { right_color_data_.clear(); }
        if (img_info_.size() > 30) { img_info_.clear(); }
      }
    }
  }
  image_color_.clear();
  img_info_.clear();
}

void CameraPrivate::OldSyntheticImageColor() {
  std::unique_lock<std::mutex> _(cap_color_mtx_);
  image_color_wait_.wait_for(_, std::chrono::seconds(1));
  if (image_color_.empty()) { return; }

  for (auto color : image_color_) {
    OldTransferColor(color);
    if (left_color_data_.size() > 30) { left_color_data_.clear(); }
    if (right_color_data_.size() > 30) { right_color_data_.clear(); }
  }
  image_color_.clear();
}

void CameraPrivate::TransferColor(Image::pointer color, img_info_data_t info) {
  if (!is_enable_image_[ImageType::IMAGE_RIGHT_COLOR]) {
    stream_data_t data;
    data.img_info = std::make_shared<ImgInfo>();
    *data.img_info = *info.img_info;
    data.img = color->Clone();
    left_color_data_.push_back(data);
  } else {
    CutPart(ImageType::IMAGE_LEFT_COLOR, color, info);
    CutPart(ImageType::IMAGE_RIGHT_COLOR, color, info);
  }
}

void CameraPrivate::OldTransferColor(Image::pointer color) {
  if (!is_enable_image_[ImageType::IMAGE_RIGHT_COLOR]) {
    stream_data_t data;
    data.img_info = nullptr;
    data.img = color->Clone();
    left_color_data_.push_back(data);
  } else {
    OldCutPart(ImageType::IMAGE_LEFT_COLOR, color);
    OldCutPart(ImageType::IMAGE_RIGHT_COLOR, color);
  }
}

void CameraPrivate::CutPart(ImageType type,
    Image::pointer color, img_info_data_t info) {
  stream_data_t data;
  data.img_info = std::make_shared<ImgInfo>();
  *data.img_info = *info.img_info;
  if (type == ImageType::IMAGE_LEFT_COLOR) {
    data.img = color->CutPart(ImageType::IMAGE_LEFT_COLOR);
    left_color_data_.push_back(data);
  } else if (type == ImageType::IMAGE_RIGHT_COLOR) {
    data.img = color->CutPart(ImageType::IMAGE_RIGHT_COLOR);
    right_color_data_.push_back(data);
  }
}

void CameraPrivate::OldCutPart(ImageType type, Image::pointer color) {
  stream_data_t data;
  data.img_info = nullptr;
  if (type == ImageType::IMAGE_LEFT_COLOR) {
    data.img = color->CutPart(ImageType::IMAGE_LEFT_COLOR);
    left_color_data_.push_back(data);
  } else if (type == ImageType::IMAGE_RIGHT_COLOR) {
    data.img = color->CutPart(ImageType::IMAGE_RIGHT_COLOR);
    right_color_data_.push_back(data);
  }
}

void CameraPrivate::SyntheticImageDepth() {
  std::unique_lock<std::mutex> _(cap_depth_mtx_);
  image_depth_wait_.wait_for(_, std::chrono::seconds(1));
  for (auto depth : image_depth_) {
    stream_data_t data;
    data.img_info = nullptr;
    data.img = depth->Clone();
    depth_data_.push_back(data);
    if (depth_data_.size() > 30) { depth_data_.clear(); }
  }
  image_depth_.clear();
}

void CameraPrivate::StartCaptureImage() {
  is_capture_image_ = true;
  cap_image_thread_ = std::thread([this]() {
    ErrorCode code = ErrorCode::SUCCESS;
    while (is_capture_image_) {
      if (is_enable_image_[ImageType::IMAGE_LEFT_COLOR] ||
          is_enable_image_[ImageType::IMAGE_RIGHT_COLOR]) {
        CaptureImageColor(&code);
      }
      if (is_enable_image_[ImageType::IMAGE_DEPTH]) {
        CaptureImageDepth(&code);
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds(1));
    }
  });
}

void CameraPrivate::StopCaptureImage() {
  is_capture_image_ = false;
  cap_image_thread_.join();
  image_color_wait_.notify_all();
  image_depth_wait_.notify_all();
}

void CameraPrivate::StartSyntheticImage() {
  is_synthetic_image_ = true;
  sync_thread_ = std::thread([this]() {
    while (is_capture_image_) {
      if (is_enable_image_[ImageType::IMAGE_LEFT_COLOR] ||
          is_enable_image_[ImageType::IMAGE_RIGHT_COLOR]) {
        if (is_hid_exist_) {
          SyntheticImageColor();
        } else {
          OldSyntheticImageColor();
        }
      }
      if (is_enable_image_[ImageType::IMAGE_DEPTH]) {
        SyntheticImageDepth();
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds(1));
    }
  });
}

void CameraPrivate::StopSyntheticImage() {
  is_synthetic_image_ = false;
  sync_thread_.join();
}

bool CameraPrivate::StartHidTracking() {
  channels_->SetImuCallback(std::bind(&CameraPrivate::ImuDataCallback,
        this, std::placeholders::_1));
  channels_->SetImgInfoCallback(std::bind(&CameraPrivate::ImageInfoCallback,
        this, std::placeholders::_1));
  if (!channels_->StartHidTracking()) {
    return false;
  }

  is_imu_open_ = true;
  return true;
}

void CameraPrivate::ImuDataCallback(const ImuDataPacket &packet) {
  auto &&imu = std::make_shared<ImuData>();
  imu->flag = packet.flag;
  imu->temperature = static_cast<double>(packet.temperature * 0.125 + 23);
  imu->timestamp = packet.timestamp;

  if (imu->flag == 1) {
    imu->accel[0] = packet.accel_or_gyro[0] * 12.f / 0x10000;
    imu->accel[1] = packet.accel_or_gyro[1] * 12.f / 0x10000;
    imu->accel[2] = packet.accel_or_gyro[2] * 12.f / 0x10000;
    imu->gyro[0] = 0;
    imu->gyro[1] = 0;
    imu->gyro[2] = 0;
  } else if (imu->flag == 2) {
    imu->accel[0] = 0;
    imu->accel[1] = 0;
    imu->accel[2] = 0;
    imu->gyro[0] = packet.accel_or_gyro[0] * 2000.f / 0x10000;
    imu->gyro[1] = packet.accel_or_gyro[1] * 2000.f / 0x10000;
    imu->gyro[2] = packet.accel_or_gyro[2] * 2000.f / 0x10000;
  } else {
    LOGW("Unaccpected imu, flag=%d is wrong", imu->flag);
    return;
  }

  if (is_process_mode_[ProcessMode::ASSEMBLY]) {
    ScaleAssemCompensate(imu);
  } else if (is_process_mode_[ProcessMode::WARM_DRIFT]) {
    TempCompensate(imu);
  } else if (is_process_mode_[ProcessMode::ALL]) {
    TempCompensate(imu);
    ScaleAssemCompensate(imu);
  }

  ++motion_count_;
  if (motion_count_ > 20) {
    motion_data_t tmp = {imu};
    cache_imu_data_.push_back(tmp);
    std::lock_guard<std::mutex> _(mtx_imu_);
    imu_data_.insert(imu_data_.end(), cache_imu_data_.begin(),
        cache_imu_data_.end());
    cache_imu_data_.clear();
  }
}

void CameraPrivate::ImageInfoCallback(const ImgInfoPacket &packet) {
  auto &&img_info = std::make_shared<ImgInfo>();

  img_info->frame_id = packet.frame_id;
  img_info->timestamp = packet.timestamp;
  img_info->exposure_time = packet.exposure_time;

  img_info_data_t tmp = {img_info};
  cache_image_info_.push_back(tmp);

  std::lock_guard<std::mutex> _(mtx_img_info_);
  img_info_.insert(img_info_.end(), cache_image_info_.begin(), cache_image_info_.end());
  cache_image_info_.clear();
}

std::vector<MotionData> CameraPrivate::GetImuDatas() {
  if (!is_imu_open_)
    LOGE("Imu is not opened !");

  std::lock_guard<std::mutex> _(mtx_imu_);
  motion_datas_t tmp = imu_data_;
  imu_data_.clear();
  return tmp;
}

void CameraPrivate::EnableImageType(const ImageType& type) {
  switch (type) {
    case ImageType::IMAGE_LEFT_COLOR:
      is_enable_image_[type] = true;
      break;
    case ImageType::IMAGE_RIGHT_COLOR:
      is_enable_image_[type] = true;
      stream_mode_ = StreamMode::STREAM_2560x720;
      break;
    case ImageType::IMAGE_DEPTH:
      is_enable_image_[type] = true;
      break;
    case ImageType::ALL:
      EnableImageType(ImageType::IMAGE_LEFT_COLOR);
      EnableImageType(ImageType::IMAGE_RIGHT_COLOR);
      EnableImageType(ImageType::IMAGE_DEPTH);
      break;
    default:
      LOGE("EnableImageType:: ImageType is unknown.");
  }
}

void CameraPrivate::ReadAllInfos() {
  device_params_ = std::make_shared<DeviceParams>();

  Channels::imu_params_t imu_params;
  if (!channels_->GetFiles(device_params_.get(), &imu_params)) {
    LOGE("%s %d:: Read device infos failed. Please upgrade"
        "your firmware to the latest version.", __FILE__, __LINE__);
    return;
  }

  LOGI("\nDevice info: name: %s", device_params_->name.c_str());
  LOGI("             serial_number: %s", device_params_->serial_number.c_str());
  LOGI("             firmware_version: %s",
      device_params_->firmware_version.to_string().c_str());
  LOGI("             hardware_version: %s",
      device_params_->hardware_version.to_string().c_str());
  LOGI("             spec_version: %s",
      device_params_->spec_version.to_string().c_str());
  LOGI("             lens_type: %s",
      device_params_->lens_type.to_string().c_str());
  LOGI("             imu_type: %s",
      device_params_->imu_type.to_string().c_str());
  LOGI("             nominal_baseline: %u", device_params_->nominal_baseline);

  if (imu_params.ok) {
    SetMotionIntrinsics({imu_params.in_accel, imu_params.in_gyro});
    SetMotionExtrinsics(imu_params.ex_left_to_imu);
    // std::cout << GetMotionIntrinsics() << std::endl;
    // std::cout << GetMotionExtrinsics() << std::endl;
  } else {
    LOGE("%s %d:: Motion intrinsics & extrinsics not exist",
        __FILE__, __LINE__);
  }
}

void CameraPrivate::EnableImuProcessMode(const ProcessMode &mode) {
  switch (mode) {
    case ProcessMode::ASSEMBLY:
      is_process_mode_[mode] = true;
      break;
    case ProcessMode::WARM_DRIFT:
      is_process_mode_[mode] = true;
      break;
    case ProcessMode::ALL:
      is_process_mode_[mode] = true;
      break;
    default:
      break;
  }
}

void CameraPrivate::TempCompensate(std::shared_ptr<ImuData> data) {
  if (nullptr == motion_intrinsics_) {
    return;
  }

  double temp = data->temperature;
  if (data->flag == 1) {
    data->accel[0] -= motion_intrinsics_->accel.x[1] * temp
      + motion_intrinsics_->accel.x[0];
    data->accel[1] -= motion_intrinsics_->accel.y[1] * temp
      + motion_intrinsics_->accel.y[0];
    data->accel[2] -= motion_intrinsics_->accel.z[1] * temp
      + motion_intrinsics_->accel.z[0];
  } else if (data->flag == 2) {
    data->gyro[0] -= motion_intrinsics_->gyro.x[1] * temp
      + motion_intrinsics_->gyro.x[0];
    data->gyro[1] -= motion_intrinsics_->gyro.y[1] * temp
      + motion_intrinsics_->gyro.y[0];
    data->gyro[2] -= motion_intrinsics_->gyro.z[1] * temp
      + motion_intrinsics_->gyro.z[0];
  }
}

void CameraPrivate::ScaleAssemCompensate(std::shared_ptr<ImuData> data) {
  if (nullptr == motion_intrinsics_) {
    return;
  }

  double dst[3][3] = {0};
  if (data->flag == 1) {
    matrix_3x3(motion_intrinsics_->accel.scale,
        motion_intrinsics_->accel.assembly, dst);
    double s[3][1] = {0};
    double d[3][1] = {0};
    for (int i = 0; i < 3; i++) {
      s[i][0] = data->accel[i];
    }
    matrix_3x1(dst, s, d);
    for (int i = 0; i < 3; i++) {
      data->accel[i] = d[i][0];
    }
  } else if (data->flag == 2) {
    matrix_3x3(motion_intrinsics_->gyro.scale,
        motion_intrinsics_->gyro.assembly, dst);
    double s[3][1] = {0};
    double d[3][1] = {0};
    for (int i = 0; i < 3; i++) {
      s[i][0] = data->gyro[i];
    }
    matrix_3x1(dst, s, d);
    for (int i = 0; i < 3; i++) {
      data->gyro[i] = d[i][0];
    }
  }
}

void CameraPrivate::IsHidExist() {
  is_hid_exist_ = channels_->IsHidExist();
}

Image::pointer CameraPrivate::RetrieveImageColor(ErrorCode* code) {
  auto image_ptr = device_->GetImageColor();
  *code = (image_ptr == nullptr) ? ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED
      : ErrorCode::SUCCESS;
  return image_ptr;
}

Image::pointer CameraPrivate::RetrieveImageDepth(ErrorCode* code) {
  auto image_ptr = device_->GetImageDepth();
  *code = (image_ptr == nullptr) ? ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED
      : ErrorCode::SUCCESS;
  return image_ptr;
}
