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
#include "mynteyed/device/device.h"

#ifdef MYNTEYE_OS_WIN

#include "mynteyed/device/convertor.h"
#include "mynteyed/util/log.h"

#include "colorizer_win.h"

MYNTEYE_USE_NAMESPACE

void Device::OnInit() {
  is_color_ok_ = false;
  is_depth_ok_ = false;
  colorizer_ = std::make_shared<ColorizerLinux>();
}

void Device::ImgCallback(EtronDIImageType::Value imgType, int imgId,
      unsigned char* imgBuf, int imgSize, int width, int height,
      int serialNumber, void *pParam) {
  Device* p = static_cast<Device*>(pParam);

  if (EtronDIImageType::IsImageColor(imgType)) {
    std::lock_guard<std::mutex> _(p->color_mtx_);
    // LOGI("Image callback color");
    if (!p->color_image_buf_) {
      unsigned int color_img_width  =(unsigned int)(
          p->stream_color_info_ptr_[p->color_res_index_].nWidth);
      unsigned int color_img_height =(unsigned int)(
          p->stream_color_info_ptr_[p->color_res_index_].nHeight);

      /*
      if (imgType == EtronDIImageType::COLOR_RGB24) {
        p->color_image_buf_ = ImageColor::Create(ImageFormat::COLOR_RGB,
            color_img_width, color_img_height, true);
      */
      if (imgType == EtronDIImageType::COLOR_YUY2) {
        p->color_image_buf_ = ImageColor::Create(ImageFormat::COLOR_YUYV,
            color_img_width, color_img_height, true);
      } else if (imgType == EtronDIImageType::COLOR_MJPG) {
        p->color_image_buf_ = ImageColor::Create(ImageFormat::COLOR_MJPG,
            color_img_width, color_img_height, true);
      }
    } else {
      p->color_image_buf_->ResetBuffer();
    }
    p->color_image_buf_->set_valid_size(imgSize);
    p->color_image_buf_->set_frame_id(serialNumber);
    std::copy(imgBuf, imgBuf + imgSize, p->color_image_buf_->data());
    p->is_color_ok_ = true;
    p->color_condition_.notify_one();
  } else if (EtronDIImageType::IsImageDepth(imgType)) {
    std::lock_guard<std::mutex> _(p->depth_mtx_);
    // LOGI("Image callback depth");
    if (!p->depth_image_buf_) {
      unsigned int depth_img_width  = (unsigned int)(
          p->stream_depth_info_ptr_[p->depth_res_index_].nWidth);
      unsigned int depth_img_height = (unsigned int)(
          p->stream_depth_info_ptr_[p->depth_res_index_].nHeight);
      p->depth_image_buf_ = ImageDepth::Create(ImageFormat::DEPTH_RAW,
          depth_img_width, depth_img_height, true);
    } else {
      p->depth_image_buf_->ResetBuffer();
    }
    p->depth_image_buf_->set_valid_size(imgSize);
    p->depth_image_buf_->set_frame_id(serialNumber);
    std::copy(imgBuf, imgBuf + imgSize, p->depth_image_buf_->data());
    p->is_depth_ok_ = true;
    p->depth_condition_.notify_one();
  } else {
    LOGE("Image callback failed. Unknown image type.");
  }
}

Image::pointer Device::GetImageColor() {
  // LOGI("Get image color");
  std::unique_lock<std::mutex> lock(color_mtx_);
  if (!color_condition_.wait_for(lock, std::chrono::seconds(1),
      [this] { return is_color_ok_; })) {
    return nullptr;
  }
  is_color_ok_ = false;

  if (color_image_buf_) {
    // unsigned int color_img_width  = (unsigned int)(
    //     stream_color_info_ptr_[color_res_index_].nWidth);
    // unsigned int color_img_height = (unsigned int)(
    //     stream_color_info_ptr_[color_res_index_].nHeight);

    device_status_[COLOR_DEVICE] = true;
    is_actual_[COLOR_DEVICE] = true;
    if (color_image_buf_->format() == ImageFormat::COLOR_MJPG) {  // mjpg
      // return clone as it will be changed in imgcallback
      return color_image_buf_->Clone();
    } else if (color_image_buf_->format() == ImageFormat::COLOR_YUYV) {  // YUYV
      /*
      // clone as it will be changed in imgcallback
      auto color = color_image_buf_->Clone();
      // flip afer clone, because the buffer may not updated when get again
      FLIP_UP_DOWN_C3(color->data(), color_img_width, color_img_height);
      RGB_TO_BGR(color->data(), color_img_width, color_img_height);
      */
      return color_image_buf_->Clone();
    } else {
      LOGE("Unknown image color type.");
    }
  }

  return nullptr;
}

Image::pointer Device::GetImageDepth() {
  // LOGI("Get image depth");
  std::unique_lock<std::mutex> lock(depth_mtx_);
  if (!depth_condition_.wait_for(lock, std::chrono::seconds(1),
      [this] { return is_depth_ok_; })) {
        return nullptr;
  }
  is_depth_ok_ = false;

  if (depth_image_buf_) {
    device_status_[DEPTH_DEVICE] = true;
    is_actual_[DEPTH_DEVICE] = true;

    return colorizer_->Process(depth_image_buf_, depth_mode_);
  }

  return nullptr;
}

int Device::OpenDevice(const DeviceMode& dev_mode) {
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

  bool toRgb = false;
  // Depth0: none
  // Depth1: unshort
  // Depth2: ?
  int depthStreamSwitch = EtronDIDepthSwitch::Depth1;
  // 0x01: color and depth frame output synchrously, for depth map module only
  // 0x02: enable post-process, for Depth Map module only
  // 0x04: stitch images if this bit is set, for fisheye spherical module only
  // 0x08: use OpenCL in stitching. This bit effective only when bit-2 is set.
  BYTE ctrlMode = 0x01;

  switch (dev_mode) {
    case DeviceMode::DEVICE_COLOR:
      color_device_opened_ = true;
      depth_device_opened_ = false;

      return EtronDI_OpenDeviceEx(handle_, &dev_sel_info_,
        color_res_index_, toRgb,
        -1, depthStreamSwitch,
        Device::ImgCallback, this, &framerate_, ctrlMode);
    case DeviceMode::DEVICE_DEPTH:
      color_device_opened_ = false;
      depth_device_opened_ = true;

      return EtronDI_OpenDeviceEx(handle_, &dev_sel_info_,
        -1, toRgb,
        depth_res_index_, depthStreamSwitch,
        Device::ImgCallback, this, &framerate_, ctrlMode);
      break;
    case DeviceMode::DEVICE_ALL:
      color_device_opened_ = true;
      depth_device_opened_ = true;

      return EtronDI_OpenDeviceEx(handle_, &dev_sel_info_,
        color_res_index_, toRgb,
        depth_res_index_, depthStreamSwitch,
        Device::ImgCallback, this, &framerate_, ctrlMode);
    default:
      throw_error("ERROR:: DeviceMode is unknown.");
  }
}

bool Device::Restart() {
  EtronDI_CloseDevice(handle_, &dev_sel_info_);
  // EtronDI_Release(&handle_);
  EtronDI_Init(&handle_, false);
  if (!handle_) { return false; }

  // SetAutoExposureEnabled(open_params_.state_ae);
  // SetAutoWhiteBalanceEnabled(open_params_.state_awb);

  UpdateStreamInfos();
  EtronDI_SetDepthDataType(handle_, &dev_sel_info_, depth_data_type_);

  int ret = OpenDevice(open_params_.dev_mode);
  if (ret != ETronDI_OK) {
    LOGE("%s, %d:: Reopen device failed.", __FILE__, __LINE__);
    return false;
  }
  ResumeParams();

  return true;
}

#endif
