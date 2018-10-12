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

#include <stdexcept>
#include <string>
#include <chrono>

#include "mynteye/internal/camera_p.h"
#include "mynteye/internal/channels.h"
#include "mynteye/util/log.h"
#include "mynteye/util/rate.h"


MYNTEYE_USE_NAMESPACE

namespace {

void get_stream_size(const StreamMode& stream_mode, int* width, int* height) {
  switch (stream_mode) {
    case StreamMode::STREAM_1280x480:
      *width = 1280;
      *height = 480;
      break;
    case StreamMode::STREAM_1280x720:
      *width = 1280;
      *height = 720;
      break;
    case StreamMode::STREAM_2560x720:
      *width = 2560;
      *height = 720;
      break;
    // case StreamMode::STREAM_2560x960:
    //   *width = 2560;
    //   *height = 960;
    //   break;
    case StreamMode::STREAM_640x480:
      *width = 640;
      *height = 480;
      break;
    default:
      throw new std::runtime_error("StreamMode is unknown");
  }
}

std::string get_stream_format_string(const StreamFormat& stream_format) {
  switch (stream_format) {
    case StreamFormat::STREAM_MJPG:
      return "MJPG";
    case StreamFormat::STREAM_YUYV:
      return "YUYV";
    default:
      throw new std::runtime_error("StreamFormat is unknown");
  }
}

}  // namespace

inline std::uint8_t check_sum(std::uint8_t *buf, std::uint8_t length) {
  std::uint8_t crc8 = 0;
  while (length--) {
    crc8 = crc8 ^ (*buf++);
  }
  return crc8;
}

std::uint16_t CameraPrivate::counter = 1;

CameraPrivate::CameraPrivate()
  : etron_di_(nullptr),
    dev_sel_info_({-1}),
    color_res_index_(0),
    depth_res_index_(0),
    framerate_(10),
    rate_(nullptr),
    stream_info_dev_index_(-1),
    color_serial_number_(0),
    depth_serial_number_(0),
    color_image_size_(0),
    depth_image_size_(0),
    color_image_buf_(nullptr),
    depth_image_buf_(nullptr),
    depth_buf_(nullptr),
    is_capture_image_(false),
    is_imu_open_(false),
    is_start_(false) {

  DBG_LOGD(__func__);

  int ret = EtronDI_Init(&etron_di_, false);
  DBG_LOGI("MYNTEYE Init: %d", ret);
  UNUSED(ret);

  stream_color_info_ptr_ =
      (PETRONDI_STREAM_INFO)malloc(sizeof(ETRONDI_STREAM_INFO)*64);
  stream_depth_info_ptr_ =
      (PETRONDI_STREAM_INFO)malloc(sizeof(ETRONDI_STREAM_INFO)*64);
  OnInit();

  channels_ = std::make_shared<Channels>();
}

CameraPrivate::~CameraPrivate() {
  DBG_LOGD(__func__);
  free(stream_color_info_ptr_);
  free(stream_depth_info_ptr_);

  channels_->StopHidTracking();
  StopCaptureImage();
  Close();
  EtronDI_Release(&etron_di_);
}

void CameraPrivate::GetDevices(std::vector<DeviceInfo>* dev_infos) {
  if (!dev_infos) {
    LOGE("GetDevices: dev_infos is null.");
    return;
  }
  dev_infos->clear();

  int count = EtronDI_GetDeviceNumber(etron_di_);
  DBG_LOGD("EtronDI_GetDeviceNumber: %d", count);

  DEVSELINFO dev_sel_info;
  DEVINFORMATION* p_dev_info =
      (DEVINFORMATION*)malloc(sizeof(DEVINFORMATION)*count);  // NOLINT

  for (int i = 0; i < count; i++) {
    dev_sel_info.index = i;

    EtronDI_GetDeviceInfo(etron_di_, &dev_sel_info, p_dev_info+i);

    char sz_buf[256];
    int actual_length = 0;
    if (ETronDI_OK == EtronDI_GetFwVersion(
        etron_di_, &dev_sel_info, sz_buf, 256, &actual_length)) {
      DeviceInfo info;
      info.index = i;
      info.name = p_dev_info[i].strDevName;
      info.type = p_dev_info[i].nDevType;
      info.pid = p_dev_info[i].wPID;
      info.vid = p_dev_info[i].wVID;
      info.chip_id = p_dev_info[i].nChipID;
      info.fw_version = sz_buf;
      dev_infos->push_back(std::move(info));
    }
  }

  free(p_dev_info);
}

void CameraPrivate::GetResolutions(const std::int32_t& dev_index,
    std::vector<StreamInfo>* color_infos,
    std::vector<StreamInfo>* depth_infos) {
  if (!color_infos) {
    LOGE("GetResolutions: color_infos is null.");
    return;
  }
  color_infos->clear();

  if (!depth_infos) {
    LOGE("GetResolutions: depth_infos is null.");
    return;
  }
  depth_infos->clear();

  memset(stream_color_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO)*64);
  memset(stream_depth_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO)*64);

  DEVSELINFO dev_sel_info{dev_index};
  EtronDI_GetDeviceResolutionList(etron_di_, &dev_sel_info, 64,
      stream_color_info_ptr_, 64, stream_depth_info_ptr_);

  PETRONDI_STREAM_INFO stream_temp_info_ptr = stream_color_info_ptr_;
  int i = 0;
  while (i < 64) {
    if (stream_temp_info_ptr->nWidth > 0) {
      StreamInfo info;
      info.index = i;
      info.width = stream_temp_info_ptr->nWidth;
      info.height = stream_temp_info_ptr->nHeight;
      info.format = stream_temp_info_ptr->bFormatMJPG ?
          StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV;
      color_infos->push_back(info);
    }
    stream_temp_info_ptr++;
    i++;
  }

  stream_temp_info_ptr = stream_depth_info_ptr_;
  i = 0;
  while (i < 64) {
    if (stream_temp_info_ptr->nWidth > 0) {
      StreamInfo info;
      info.index = i;
      info.width = stream_temp_info_ptr->nWidth;
      info.height = stream_temp_info_ptr->nHeight;
      info.format = stream_temp_info_ptr->bFormatMJPG ?
          StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV;
      depth_infos->push_back(info);
    }
    stream_temp_info_ptr++;
    i++;
  }

  stream_info_dev_index_ = dev_index;
}

void CameraPrivate::GetResolutionIndex(const InitParams& params,
    int* color_res_index,
    int* depth_res_index) {
  return GetResolutionIndex(
      params.dev_index, params.stream_mode,
      params.color_stream_format, params.depth_stream_format,
      color_res_index, depth_res_index);
}

void CameraPrivate::GetResolutionIndex(const std::int32_t& dev_index,
    const StreamMode& stream_mode,
    const StreamFormat& color_stream_format,
    const StreamFormat& depth_stream_format,
    int *color_res_index,
    int *depth_res_index) {
  if (!color_res_index) {
    LOGE("GetResolutionIndex: color_res_index is null.");
    return;
  }
  if (!depth_res_index) {
    LOGE("GetResolutionIndex: depth_res_index is null.");
    return;
  }

  *color_res_index = -1;
  *depth_res_index = -1;

  int width = 0, height = 0;
  get_stream_size(stream_mode, &width, &height);

  memset(stream_color_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO)*64);
  memset(stream_depth_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO)*64);

  DEVSELINFO dev_sel_info{dev_index};
  EtronDI_GetDeviceResolutionList(etron_di_, &dev_sel_info, 64,
      stream_color_info_ptr_, 64, stream_depth_info_ptr_);

  PETRONDI_STREAM_INFO stream_temp_info_ptr = stream_color_info_ptr_;
  int i = 0;
  while (i < 64) {
    if (stream_temp_info_ptr->nWidth == width &&
        stream_temp_info_ptr->nHeight == height &&
        color_stream_format == (stream_temp_info_ptr->bFormatMJPG ?
            StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV)) {
      *color_res_index = i;
      break;
    }
    stream_temp_info_ptr++;
    i++;
  }

  if (*color_res_index == -1) {
    LOGE("Error: Color Mode width[%d] height[%d] format[%s] not support. "
        "Please check the resolution list.", width, height,
        get_stream_format_string(color_stream_format));
    *color_res_index = 0;
  }

  stream_temp_info_ptr = stream_depth_info_ptr_;
  i = 0;
  while (i < 64) {
    if (stream_temp_info_ptr->nHeight == height &&
        depth_stream_format == (stream_temp_info_ptr->bFormatMJPG ?
            StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV)) {
      *depth_res_index = i;
      break;
    }
    stream_temp_info_ptr++;
    i++;
  }

  if (*depth_res_index == -1) {
    LOGE("Error: Depth Mode width[%d] height[%d] format[%s] not support. "
        "Please check the resolution list.", width, height,
        get_stream_format_string(depth_stream_format));
    *depth_res_index = 0;
  }
}

ErrorCode CameraPrivate::SetAutoExposureEnabled(bool enabled) {
  bool ok;
  if (enabled) {
    ok = ETronDI_OK == EtronDI_EnableAE(etron_di_, &dev_sel_info_);
  } else {
    ok = ETronDI_OK == EtronDI_DisableAE(etron_di_, &dev_sel_info_);
  }
  if (ok) {
    LOGI("-- Auto-exposure state: %s", enabled ? "enabled" : "disabled");
  } else {
    LOGW("-- %s auto-exposure failed", enabled ? "Enable" : "Disable");
  }
  return ok ? ErrorCode::SUCCESS : ErrorCode::ERROR_FAILURE;
}

ErrorCode CameraPrivate::SetAutoWhiteBalanceEnabled(bool enabled) {
  bool ok;
  if (enabled) {
    ok = ETronDI_OK == EtronDI_EnableAWB(etron_di_, &dev_sel_info_);
  } else {
    ok = ETronDI_OK == EtronDI_DisableAWB(etron_di_, &dev_sel_info_);
  }
  if (ok) {
    LOGI("-- Auto-white balance state: %s", enabled ? "enabled" : "disabled");
  } else {
    LOGW("-- %s auto-white balance failed", enabled ? "Enable" : "Disable");
  }
  return ok ? ErrorCode::SUCCESS : ErrorCode::ERROR_FAILURE;
}

bool CameraPrivate::GetSensorRegister(int id, std::uint16_t address,
    std::uint16_t* value, int flag) {
  if (!IsOpened()) return false;
#ifdef MYNTEYE_OS_WIN
  return ETronDI_OK == EtronDI_GetSensorRegister(etron_di_, &dev_sel_info_, id,
      address, value, flag, 2);
#else
  return ETronDI_OK == EtronDI_GetSensorRegister(etron_di_, &dev_sel_info_, id,
      address, value, flag, SENSOR_BOTH);
#endif
}

bool CameraPrivate::GetHWRegister(std::uint16_t address, std::uint16_t* value,
    int flag) {
  if (!IsOpened()) return false;
  return ETronDI_OK == EtronDI_GetHWRegister(etron_di_, &dev_sel_info_,
      address, value, flag);
}

bool CameraPrivate::GetFWRegister(std::uint16_t address, std::uint16_t* value,
    int flag) {
  if (!IsOpened()) return false;
  return ETronDI_OK == EtronDI_GetFWRegister(etron_di_, &dev_sel_info_, address,
      value, flag);
}

bool CameraPrivate::SetSensorRegister(int id, std::uint16_t address,
    std::uint16_t value, int flag) {
  if (!IsOpened()) return false;
#ifdef MYNTEYE_OS_WIN
  return ETronDI_OK == EtronDI_SetSensorRegister(etron_di_, &dev_sel_info_, id,
      address, value, flag, 2);
#else
  return ETronDI_OK == EtronDI_SetSensorRegister(etron_di_, &dev_sel_info_, id,
      address, value, flag, SENSOR_BOTH);
#endif
}

bool CameraPrivate::SetHWRegister(std::uint16_t address, std::uint16_t value,
    int flag) {
  if (!IsOpened()) return false;
  return ETronDI_OK == EtronDI_SetHWRegister(etron_di_, &dev_sel_info_, address,
      value, flag);
}

bool CameraPrivate::SetFWRegister(std::uint16_t address, std::uint16_t value,
    int flag) {
  if (!IsOpened()) return false;
  return ETronDI_OK == EtronDI_SetFWRegister(etron_di_, &dev_sel_info_, address,
      value, flag);
}

ErrorCode CameraPrivate::Open(const InitParams& params) {
  dev_sel_info_.index = params.dev_index;

  // if (params.dev_info.type == PUMA) {
    depth_data_type_ = 2;  // ETronDI_DEPTH_DATA_14_BITS
    EtronDI_SetDepthDataType(etron_di_, &dev_sel_info_, depth_data_type_);
    DBG_LOGI("SetDepthDataType: %d", depth_data_type_);
  // }

  SetAutoExposureEnabled(params.state_ae);
  SetAutoWhiteBalanceEnabled(params.state_awb);

  if (params.framerate > 0) framerate_ = params.framerate;
  LOGI("-- Framerate: %d", framerate_);

  rate_.reset(new Rate(framerate_));

#ifdef MYNTEYE_OS_LINUX
  std::string dtc_name = "Unknown";
  switch (params.depth_mode) {
    case DepthMode::DEPTH_GRAY:
      dtc_ = DEPTH_IMG_GRAY_TRANSFER;
      dtc_name = "Gray";
      break;
    case DepthMode::DEPTH_COLORFUL:
      dtc_ = DEPTH_IMG_COLORFUL_TRANSFER;
      dtc_name = "Colorful";
      break;
    case DepthMode::DEPTH_RAW:
    default:
      dtc_ = DEPTH_IMG_NON_TRANSFER;
      dtc_name = "Raw";
      break;
  }
#endif
  depth_mode_ = params.depth_mode;

  if (params.dev_index != stream_info_dev_index_) {
    std::vector<StreamInfo> color_infos;
    std::vector<StreamInfo> depth_infos;
    GetResolutions(params.dev_index, &color_infos, &depth_infos);
  }

  GetResolutionIndex(params, &color_res_index_, &depth_res_index_);
  LOGI("-- Color Stream: %dx%d %s",
      stream_color_info_ptr_[color_res_index_].nWidth,
      stream_color_info_ptr_[color_res_index_].nHeight,
      stream_color_info_ptr_[color_res_index_].bFormatMJPG ? "MJPG" : "YUYV");
  LOGI("-- Depth Stream: %dx%d %s",
      stream_depth_info_ptr_[depth_res_index_].nWidth,
      stream_depth_info_ptr_[depth_res_index_].nHeight,
      stream_depth_info_ptr_[depth_res_index_].bFormatMJPG ? "MJPG" : "YUYV");

  if (depth_data_type_ != 1 && depth_data_type_ != 2) {
    throw std::runtime_error(strings::format_string(
        "Error: Depth data type (%d) not supported.", depth_data_type_));
  }

  if (params.ir_intensity >= 0) {
    if (SetFWRegister(0xE0, params.ir_intensity)) {
      LOGI("-- IR intensity: %d", params.ir_intensity);
    } else {
      LOGI("-- IR intensity: %d (failed)", params.ir_intensity);
    }
  }

  ReleaseBuf();

#ifdef MYNTEYE_OS_WIN
  // int EtronDI_OpenDeviceEx(
  //     void* pHandleEtronDI,
  //     PDEVSELINFO pDevSelInfo,
  //     int colorStreamIndex,
  //     bool toRgb,
  //     int depthStreamIndex,
  //     int depthStreamSwitch,
  //     EtronDI_ImgCallbackFn callbackFn,
  //     void* pCallbackParam,
  //     int* pFps,
  //     BYTE ctrlMode)

  bool toRgb = true;
  // Depth0: none
  // Depth1: unshort
  // Depth2: ?
  int depthStreamSwitch = EtronDIDepthSwitch::Depth1;
  // 0x01: color and depth frame output synchrously, for depth map module only
  // 0x02: enable post-process, for Depth Map module only
  // 0x04: stitch images if this bit is set, for fisheye spherical module only
  // 0x08: use OpenCL in stitching. This bit effective only when bit-2 is set.
  BYTE ctrlMode = 0x01;

  int ret = EtronDI_OpenDeviceEx(etron_di_, &dev_sel_info_,
      color_res_index_, toRgb,
      depth_res_index_, depthStreamSwitch,
      CameraPrivate::ImgCallback, this, &framerate_, ctrlMode);
#else
  int ret = EtronDI_OpenDevice2(etron_di_, &dev_sel_info_,
      stream_color_info_ptr_[color_res_index_].nWidth,
      stream_color_info_ptr_[color_res_index_].nHeight,
      stream_color_info_ptr_[color_res_index_].bFormatMJPG,
      stream_depth_info_ptr_[depth_res_index_].nWidth,
      stream_depth_info_ptr_[depth_res_index_].nHeight,
      dtc_, false, NULL, &framerate_);
#endif

  if (ETronDI_OK == ret) {
    StartCaptureImage();
    return ErrorCode::SUCCESS;
  } else {
    dev_sel_info_.index = -1;  // reset flag
    return ErrorCode::ERROR_CAMERA_OPEN_FAILED;
  }
}

bool CameraPrivate::IsOpened() const {
  return dev_sel_info_.index != -1;
}

void CameraPrivate::CheckOpened() const {
  if (!IsOpened()) throw std::runtime_error("Error: Camera not opened.");
}

/*
Image::pointer CameraPrivate::RetrieveImage(const ImageType& type,
    ErrorCode* code) {
  if (!IsOpened()) {
    *code = ErrorCode::ERROR_CAMERA_NOT_OPENED;
    return nullptr;
  }
  switch (type) {
    case ImageType::IMAGE_COLOR:
      return RetrieveImageColor(code);
    case ImageType::IMAGE_DEPTH:
      return RetrieveImageDepth(code);
    default:
      throw new std::runtime_error("RetrieveImage: ImageType is unknown");
  }
}
*/

std::vector<device::StreamData> CameraPrivate::RetrieveImage(const ImageType& type,
    ErrorCode* code) {
  if (!IsOpened()) {
    *code = ErrorCode::ERROR_CAMERA_NOT_OPENED;
    return {};
  }

  switch (type) {
    case ImageType::IMAGE_COLOR: {
      std::lock_guard<std::mutex> _(cap_color_mtx_);
      stream_datas_t data = color_data_;
      color_data_.clear();
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
    case ImageType::IMAGE_COLOR: {
      std::lock_guard<std::mutex> _(cap_color_mtx_);
      if (color_data_.empty()) {
        return {};
      }
      auto data = color_data_.back();
      color_data_.clear();
      return data;
    } break;
    case ImageType::IMAGE_DEPTH: {
      std::lock_guard<std::mutex> _(cap_depth_mtx_);
      if (depth_data_.empty()) {
        return {};
      }
      auto data = depth_data_.back();
      depth_data_.clear();
      return data;
    } break;
    default:
      throw new std::runtime_error("RetrieveImage: ImageType is unknown");
  }
}

void CameraPrivate::CaptureImage(const ImageType& type,
    ErrorCode* code) {
  switch (type) {
    case ImageType::IMAGE_COLOR: {
      std::unique_lock<std::mutex> _(cap_color_mtx_);
      auto p = RetrieveImageColor(code);
      if (p) {
        auto color = p->Clone();
        image_color_.push_back(color);
      }
      if (!image_color_.empty()) {
        image_color_wait_.notify_one();
      }
    } break;
    case ImageType::IMAGE_DEPTH: {
      std::unique_lock<std::mutex> _(cap_depth_mtx_);
      auto p = RetrieveImageDepth(code);
      if (p) {
        auto depth = p->Clone();
        image_depth_.push_back(depth);
      }
      if (!image_depth_.empty()) {
        image_depth_wait_.notify_one();
      }
    } break;
    case ImageType::ALL: {
      CaptureImage(ImageType::IMAGE_COLOR, code);
      CaptureImage(ImageType::IMAGE_DEPTH, code);
    } break;
    default:
      throw new std::runtime_error("CaptureImage: ImageType is unknown");
  }
}

void CameraPrivate::Synthetic(const ImageType &type) {
  switch (type) {
    case ImageType::IMAGE_COLOR: {
      std::unique_lock<std::mutex> _(cap_color_mtx_);
      image_color_wait_.wait_for(_, std::chrono::seconds(1));
      for (auto color : image_color_) {
        for (auto info : img_info_) {
          if (color->frame_id() == info.img_info->frame_id) {
            stream_data_t data;
            data.img_info = std::make_shared<ImgInfo>();
            *data.img_info = *info.img_info;
            data.img = color->Clone();
            color_data_.push_back(data);
          } else if (color->frame_id() < info.img_info->frame_id) {
            image_color_.clear();
            return;
          } else {
            img_info_.clear();
            return;
          }
        }
      }
      image_color_.clear();
      img_info_.clear();
    } break;
    case ImageType::IMAGE_DEPTH: {
      std::unique_lock<std::mutex> _(cap_depth_mtx_);
      image_depth_wait_.wait_for(_, std::chrono::seconds(1));
      for (auto depth : image_depth_) {
        stream_data_t data;
        data.img_info = nullptr;
        data.img = depth->Clone();
        depth_data_.push_back(data);
      }
      image_depth_.clear();
    } break;
    case ImageType::ALL: {
      Synthetic(ImageType::IMAGE_COLOR);
      Synthetic(ImageType::IMAGE_DEPTH);
    } break;
    default:
      throw new std::runtime_error("Synthetic: ImageType is unknown");
  }
}

void CameraPrivate::StartCaptureImage() {
  is_capture_image_ = true;
  cap_image_thread_ = std::thread([this]() {
    ErrorCode code = ErrorCode::SUCCESS;
    while (is_capture_image_) {
      CaptureImage(ImageType::ALL, &code);
    }
  });
  sync_thread_ = std::thread([this]() {
    while (is_capture_image_) {
      Synthetic(ImageType::ALL);
    }
  });
}

void CameraPrivate::StopCaptureImage() {
  is_capture_image_ = false;
  if (cap_image_thread_.joinable()) {
    cap_image_thread_.join();
  }
  if (sync_thread_.joinable()) {
    sync_thread_.join();
  }
}

void CameraPrivate::Wait() {
  if (rate_) {
    OnPreWait();
    rate_->Sleep();
    OnPostWait();
  }
}

void CameraPrivate::Close() {
  if (dev_sel_info_.index != -1) {
    StopCaptureImage();
    channels_->StopHidTracking();
    EtronDI_CloseDevice(etron_di_, &dev_sel_info_);
    dev_sel_info_.index = -1;
  }
  ReleaseBuf();
}

void CameraPrivate::ReleaseBuf() {
  color_image_buf_ = nullptr;
  depth_image_buf_ = nullptr;
  if (!depth_buf_) {
    delete depth_buf_;
    depth_buf_ = nullptr;
  }
}

ErrorCode CameraPrivate::StartHidTracking() {
  channels_->SetImuCallback(std::bind(&CameraPrivate::ImuDataCallback,
        this, std::placeholders::_1));
  channels_->SetImgInfoCallback(std::bind(&CameraPrivate::ImageInfoCallback,
        this, std::placeholders::_1));
  is_imu_open_ = true;
  channels_->StartHidTracking();

  return ErrorCode::SUCCESS;
}

void CameraPrivate::ImuDataCallback(const ImuPacket &packet) {
  for (auto &&seg : packet.segments) {
    auto &&imu = std::make_shared<ImuData>();
    imu->flag = seg.flag;
    imu->temperature = static_cast<double>((std::uint16_t)(seg.temperature * 0.125 + 23));
    imu->timestamp = seg.timestamp;

    if (imu->flag == 1) {
      imu->accel[0] = (std::int16_t)seg.accel_or_gyro[0] * 12.f / 0x10000;
      imu->accel[1] = (std::int16_t)seg.accel_or_gyro[1] * 12.f / 0x10000;
      imu->accel[2] = (std::int16_t)seg.accel_or_gyro[2] * 12.f / 0x10000;
      imu->gyro[0] = 0;
      imu->gyro[1] = 0;
      imu->gyro[2] = 0;
    } else if (imu->flag == 2) {
      imu->accel[0] = 0;
      imu->accel[1] = 0;
      imu->accel[2] = 0;
      imu->gyro[0] = (std::int16_t)seg.accel_or_gyro[0] * 2000.f / 0x10000;
      imu->gyro[1] = (std::int16_t)seg.accel_or_gyro[1] * 2000.f / 0x10000;
      imu->gyro[2] = (std::int16_t)seg.accel_or_gyro[2] * 2000.f / 0x10000;
    } else {
      imu->Reset();
    }

    std::lock_guard<std::mutex> _(mtx_imu_);
    motion_data_t tmp = {imu};
    imu_data_.push_back(tmp);
  }
}

void CameraPrivate::ImageInfoCallback(const ImgInfoPacket &packet) {
  auto &&img_info = std::make_shared<ImgInfo>();

  img_info->frame_id = packet.frame_id;
  img_info->timestamp = packet.timestamp;
  img_info->exposure_time = packet.exposure_time;

  std::lock_guard<std::mutex> _(mtx_img_info_);
  img_info_data_t tmp = {img_info};
  img_info_.push_back(tmp);
}

std::vector<device::MotionData> CameraPrivate::GetImuData() {
  if (!is_imu_open_)
    LOGE("Imu is not opened !");

  std::lock_guard<std::mutex> _(mtx_imu_);
  motion_datas_t tmp = imu_data_;
  imu_data_.clear();
  return tmp;
}
