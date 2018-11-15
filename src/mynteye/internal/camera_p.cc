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
#include "mynteye/internal/camera_p.h"

#include <stdexcept>
#include <string>
#include <chrono>
#include <utility>

#include "mynteye/data/channels.h"
#include "mynteye/device/device.h"
#include "mynteye/internal/image_utils.h"
#include "mynteye/internal/motions.h"
#include "mynteye/util/log.h"
#include "mynteye/util/rate.h"
#include "mynteye/util/times.h"

MYNTEYE_USE_NAMESPACE

CameraPrivate::CameraPrivate() : device_(std::make_shared<Device>()) {
  DBG_LOGD(__func__);
  Init();
}

void CameraPrivate::Init() {
  is_enable_image_ = {{ImageType::IMAGE_LEFT_COLOR, false},
                      {ImageType::IMAGE_RIGHT_COLOR, false},
                      {ImageType::IMAGE_DEPTH, false}};

  channels_ = std::make_shared<Channels>();
  if (channels_->IsAvaliable()) {
    ReadDeviceFlash();
  }
  motions_ = std::make_shared<Motions>();
}

CameraPrivate::~CameraPrivate() {
  DBG_LOGD(__func__);
  Close();
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
    StartDataTracking();
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

std::shared_ptr<device::Descriptors> CameraPrivate::GetDescriptors() const {
  return descriptors_;
}

std::string CameraPrivate::GetDescriptor(const Descriptor &desc) const {
  if (!descriptors_) {
    LOGE("%s %d:: Device information not found", __FILE__, __LINE__);
    return "";
  }
  switch (desc) {
    case Descriptor::DEVICE_NAME:
      return descriptors_->name;
    case Descriptor::SERIAL_NUMBER:
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
    const StreamMode& stream_mode) {
  if (!stream_intrinsics_) {
    stream_intrinsics_ = std::make_shared<StreamIntrinsics>();
  }
  auto streamData = GetCameraCalibration(stream_mode);
  stream_intrinsics_->left.width = streamData.InImgWidth/2;
  stream_intrinsics_->left.height = streamData.InImgHeight;
  stream_intrinsics_->left.fx = streamData.CamMat1[0];
  stream_intrinsics_->left.fy = streamData.CamMat1[4];
  stream_intrinsics_->left.cx = streamData.CamMat1[2];
  stream_intrinsics_->left.cy = streamData.CamMat1[5];
  for (int i = 0; i < 5; i++) {
    stream_intrinsics_->left.coeffs[i] = streamData.CamDist1[i];
  }

  stream_intrinsics_->right.width = streamData.InImgWidth/2;
  stream_intrinsics_->right.height = streamData.InImgHeight;
  stream_intrinsics_->right.fx = streamData.CamMat2[0];
  stream_intrinsics_->right.fy = streamData.CamMat2[4];
  stream_intrinsics_->right.cx = streamData.CamMat2[2];
  stream_intrinsics_->right.cy = streamData.CamMat2[5];
  for (int i = 0; i < 5; i++) {
    stream_intrinsics_->right.coeffs[i] = streamData.CamDist2[i];
  }
  return *stream_intrinsics_;
}

StreamExtrinsics CameraPrivate::GetStreamExtrinsics(
  const StreamMode& stream_mode) {
  if (!stream_extrinsics_) {
    stream_extrinsics_ = std::make_shared<StreamExtrinsics>();
  }
  auto streamData = GetCameraCalibration(stream_mode);
  for (int i = 0; i < 9; i++) {
    stream_extrinsics_->rotation[i/3][i%3] = streamData.RotaMat[i];
  }
  for (int j = 0; j < 3; j++) {
    stream_extrinsics_->translation[j] = streamData.TranMat[j];
  }
  return *stream_extrinsics_;
}

bool CameraPrivate::WriteCameraCalibrationBinFile(const std::string& filename) {
  return device_->SetCameraCalibrationBinFile(filename);
}

MotionIntrinsics CameraPrivate::GetMotionIntrinsics() const {
  if (motion_intrinsics_) {
    return *motion_intrinsics_;
  } else {
    LOGE("Error: Motion intrinsics not found");
    return {};
  }
}

MotionExtrinsics CameraPrivate::GetMotionExtrinsics() const {
  if (motion_extrinsics_) {
    return *motion_extrinsics_;
  } else {
    LOGE("Error: Motion extrinsics not found");
    return {};
  }
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

void CameraPrivate::EnableMotionDatas(std::size_t max_size) {
  motions_->EnableMotionDatas(std::move(max_size));
  StartDataTracking();
}

std::vector<MotionData> CameraPrivate::GetMotionDatas() {
  return std::move(motions_->GetMotionDatas());
}

void CameraPrivate::Close() {
  if (device_->IsOpened()) {
    StopCaptureImage();
    StopSyntheticImage();
    StopDataTracking();
  }
  device_->Close();
}

CameraCalibration CameraPrivate::GetCameraCalibration(
    const StreamMode& stream_mode) {
  return device_->GetCameraCalibration(stream_mode);
}

void CameraPrivate::GetCameraCalibrationFile(
    const StreamMode& stream_mode, const std::string& filename) {
  return device_->GetCameraCalibrationFile(stream_mode, filename);
}

// ...

void CameraPrivate::ReadDeviceFlash() {
  if (!channels_->IsAvaliable()) {
    LOGW("Data channel is unavaliable, could not read device datas.");
    return;
  }
  descriptors_ = std::make_shared<device::Descriptors>();

  Channels::imu_params_t imu_params;
  if (!channels_->GetFiles(descriptors_.get(), &imu_params)) {
    LOGE("%s %d:: Read device descriptors failed. Please upgrade"
         "your firmware to the latest version.", __FILE__, __LINE__);
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
    // std::cout << GetMotionIntrinsics() << std::endl;
    // std::cout << GetMotionExtrinsics() << std::endl;
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
  // if (!IsOpened()) return false;  // ensure start after opened
  // if (!motions_->IsMotionDatasEnabled() && ..) return false;
  if (channels_->IsHidTracking()) return true;

  if (!channels_->IsHidAvaliable()) {
    LOGW("Data channel is unavaliable, could not track device datas.");
    return false;
  }

  channels_->SetImuDataCallback(std::bind(&Motions::ImuDataCallback,
        motions_, std::placeholders::_1));
  channels_->SetImgInfoCallback(std::bind(&CameraPrivate::ImageInfoCallback,
        this, std::placeholders::_1));

  return channels_->StartHidTracking();
}

void CameraPrivate::StopDataTracking() {
  if (channels_->IsHidTracking()) {
    channels_->StopHidTracking();
  }
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
    data.img = images::split_left_color(color);
    left_color_data_.push_back(data);
  } else if (type == ImageType::IMAGE_RIGHT_COLOR) {
    data.img = images::split_right_color(color);
    right_color_data_.push_back(data);
  }
}

void CameraPrivate::OldCutPart(ImageType type, Image::pointer color) {
  stream_data_t data;
  data.img_info = nullptr;
  if (type == ImageType::IMAGE_LEFT_COLOR) {
    data.img = images::split_left_color(color);
    left_color_data_.push_back(data);
  } else if (type == ImageType::IMAGE_RIGHT_COLOR) {
    data.img = images::split_right_color(color);
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
        if (channels_->IsAvaliable()) {
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
