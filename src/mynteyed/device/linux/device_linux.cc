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

#ifdef MYNTEYE_OS_LINUX

#include <algorithm>

#include "mynteyed/device/convertor.h"
#include "mynteyed/util/log.h"

// #define Z14_FAR  16383
#define Z14_NEAR 0
#define D11_FAR  0
#define D11_NEAR 2047
#define D8_FAR   0
#define D8_NEAR  255

MYNTEYE_USE_NAMESPACE

static const int MAX_FAILED_COUNT = 150;

void Device::OnInit() {
  dtc_ = DEPTH_IMG_NON_TRANSFER;
}

// int ret = EtronDI_Get2Image(handle_, &dev_sel_info_,
//     (BYTE*)color_img_buf_, (BYTE*)depth_img_buf_,
//     &color_image_size_, &depth_image_size_,
//     &color_serial_number_, &depth_serial_number_, depth_data_type_);

Image::pointer Device::GetImageColor() {
  unsigned int color_img_width  = (unsigned int)(
      stream_color_info_ptr_[color_res_index_].nWidth);
  unsigned int color_img_height = (unsigned int)(
      stream_color_info_ptr_[color_res_index_].nHeight);
  bool is_mjpeg = stream_color_info_ptr_[color_res_index_].bFormatMJPG;

  if (!color_image_buf_) {
    color_image_buf_ = ImageColor::Create(
      is_mjpeg ? ImageFormat::COLOR_MJPG : ImageFormat::COLOR_YUYV,
      color_img_width, color_img_height, true);
  } else {
    color_image_buf_->ResetBuffer();
  }

  int ret = EtronDI_GetColorImage(handle_, &dev_sel_info_,
      color_image_buf_->data(), &color_image_size_, &color_serial_number_, 0);

  if (ETronDI_OK != ret) {
    DBG_LOGI("GetImageColor: %d", ret);
    return nullptr;
  }
  device_status_[COLOR_DEVICE] = true;
  is_actual_[COLOR_DEVICE] = true;

  if (ir_depth_only_enabled_) {
    if (color_ir_depth_only_enabled_ &&
        (color_serial_number_ % 2) > 0) {
      return nullptr;
    } else if (!color_ir_depth_only_enabled_ &&
        (color_serial_number_ % 2) == 0) {
      return nullptr;
    }
  }

  color_image_buf_->set_valid_size(color_image_size_);
  color_image_buf_->set_frame_id(color_serial_number_);

  return color_image_buf_;
}

Image::pointer Device::GetImageDepth() {
  unsigned int depth_img_width  = (unsigned int)(
      stream_depth_info_ptr_[depth_res_index_].nWidth);
  unsigned int depth_img_height = (unsigned int)(
      stream_depth_info_ptr_[depth_res_index_].nHeight);

  bool depth_raw;
  if (dtc_ == DEPTH_IMG_COLORFUL_TRANSFER ||
      dtc_ == DEPTH_IMG_GRAY_TRANSFER) {
    depth_raw = false;

    if (depth_data_type_ == ETronDI_DEPTH_DATA_8_BITS ||
        depth_data_type_ == ETronDI_DEPTH_DATA_8_BITS_RAW) {
      depth_img_width = depth_img_width * 2;
    }

    if (!depth_image_buf_) {
      depth_buf_ = (unsigned char*)calloc(
          depth_img_width * depth_img_height * 2, sizeof(unsigned char));

      if (dtc_ == DEPTH_IMG_COLORFUL_TRANSFER) {
        depth_image_buf_ = ImageDepth::Create(ImageFormat::DEPTH_RGB,
            depth_img_width, depth_img_height, true);
      } else {  // DEPTH_IMG_GRAY_TRANSFER
        depth_image_buf_ = ImageDepth::Create(ImageFormat::DEPTH_GRAY_24,
            depth_img_width, depth_img_height, true);
      }
    } else {
      depth_image_buf_->ResetBuffer();
    }
  } else {  // DEPTH_IMG_NON_TRANSFER
    depth_raw = true;
    if (!depth_image_buf_) {
      depth_image_buf_ = ImageDepth::Create(ImageFormat::DEPTH_RAW,
          depth_img_width, depth_img_height, true);
    } else {
      depth_image_buf_->ResetBuffer();
    }
  }

  int ret = EtronDI_GetDepthImage(handle_, &dev_sel_info_,
      depth_raw ? depth_image_buf_->data() : depth_buf_,
      &depth_image_size_, &depth_serial_number_, depth_data_type_);

  if (ETronDI_OK != ret) {
    DBG_LOGI("GetImageDepth: %d", ret);
    return nullptr;
  }
  device_status_[DEPTH_DEVICE] = true;
  is_actual_[DEPTH_DEVICE] = true;

  if (ir_depth_only_enabled_) {
    if (depth_ir_depth_only_enabled_ &&
        (depth_serial_number_ % 2) > 0) {
      return nullptr;
    } else if (!depth_ir_depth_only_enabled_ &&
        (depth_serial_number_ % 2) == 0) {
      return nullptr;
    }
  }

  depth_image_buf_->set_frame_id(depth_serial_number_);

  if (depth_raw) {
    return depth_image_buf_;
  } else {
    if (dtc_ == DEPTH_IMG_COLORFUL_TRANSFER) {
      if (depth_data_type_ == ETronDI_DEPTH_DATA_14_BITS ||
          depth_data_type_ == ETronDI_DEPTH_DATA_14_BITS_RAW) {
        ColorPaletteGenerator::UpdateZ14DisplayImage_DIB24(
            m_ColorPaletteZ14, depth_buf_, depth_image_buf_->data(),
            depth_img_width, depth_img_height);
      } else if (depth_data_type_ == ETronDI_DEPTH_DATA_11_BITS ||
                 depth_data_type_ == ETronDI_DEPTH_DATA_11_BITS_RAW) {
        ColorPaletteGenerator::UpdateD11DisplayImage_DIB24(
            m_ColorPaletteD11, depth_buf_, depth_image_buf_->data(),
            depth_img_width, depth_img_height);
      } else if (depth_data_type_ == ETronDI_DEPTH_DATA_8_BITS ||
          depth_data_type_ == ETronDI_DEPTH_DATA_8_BITS_RAW) {
        ColorPaletteGenerator::UpdateD8bitsDisplayImage_DIB24(
            m_ColorPalette, depth_buf_, depth_image_buf_->data(),
            depth_img_width, depth_img_height);
      }
    } else {  // DEPTH_IMG_GRAY_TRANSFER
      if (depth_data_type_ == ETronDI_DEPTH_DATA_14_BITS ||
          depth_data_type_ == ETronDI_DEPTH_DATA_14_BITS_RAW) {
        ColorPaletteGenerator::UpdateZ14DisplayImage_DIB24(
            m_GrayPaletteZ14, depth_buf_, depth_image_buf_->data(),
            depth_img_width, depth_img_height);
      } else if (depth_data_type_ == ETronDI_DEPTH_DATA_11_BITS ||
                 depth_data_type_ == ETronDI_DEPTH_DATA_11_BITS_RAW) {
        ColorPaletteGenerator::UpdateD11DisplayImage_DIB24(
            m_GrayPaletteD11, depth_buf_, depth_image_buf_->data(),
            depth_img_width, depth_img_height);
      } else if (depth_data_type_ == ETronDI_DEPTH_DATA_8_BITS) {
        ColorPaletteGenerator::UpdateD8bitsDisplayImage_DIB24(
            m_GrayPalette, depth_buf_, depth_image_buf_->data(),
            depth_img_width, depth_img_height);
      }
    }
    return depth_image_buf_;
  }
}

int Device::OpenDevice(const DeviceMode& dev_mode) {
  int frame_rate = framerate_;

  switch (dev_mode) {
    case DeviceMode::DEVICE_COLOR:
      color_device_opened_ = true;
      depth_device_opened_ = false;

      return EtronDI_OpenDevice2(handle_, &dev_sel_info_,
          stream_color_info_ptr_[color_res_index_].nWidth,
          stream_color_info_ptr_[color_res_index_].nHeight,
          stream_color_info_ptr_[color_res_index_].bFormatMJPG,
          0, 0, dtc_, false, NULL, &frame_rate);
      break;
    case DeviceMode::DEVICE_DEPTH:
      color_device_opened_ = false;
      depth_device_opened_ = true;

      return EtronDI_OpenDevice2(handle_, &dev_sel_info_,
          0, 0, false, stream_depth_info_ptr_[depth_res_index_].nWidth,
          stream_depth_info_ptr_[depth_res_index_].nHeight,
          DEPTH_IMG_NON_TRANSFER, false, NULL, &frame_rate);
      break;
    case DeviceMode::DEVICE_ALL:
      color_device_opened_ = true;
      depth_device_opened_ = true;

      return EtronDI_OpenDevice2(handle_, &dev_sel_info_,
          stream_color_info_ptr_[color_res_index_].nWidth,
          stream_color_info_ptr_[color_res_index_].nHeight,
          stream_color_info_ptr_[color_res_index_].bFormatMJPG,
          stream_depth_info_ptr_[depth_res_index_].nWidth,
          stream_depth_info_ptr_[depth_res_index_].nHeight,
          DEPTH_IMG_NON_TRANSFER, false, NULL, &frame_rate);
      break;
    default:
      throw_error("ERROR:: DeviceMode is unknown.");
  }
}

void Device::OnInitColorPalette(const float &z14_Far) {
  float m_zFar = z14_Far;
  float m_zNear = Z14_NEAR;
  float m_d11Far = D11_FAR;
  float m_d11Near = D11_NEAR;
  float m_d8Far = D8_FAR;
  float m_d8Near = D8_NEAR;
  /*
     float m_zFar = far_;
     float m_zNear = near_;
     float m_d11Far = far_;
     float m_d11Near = near_;
     float m_d8Far = far_;
     float m_d8Near = near_;

     float m_zFar = far_;
     float m_zNear = near_;
     */
  int m_nDepthColorMapMode = 4;  // for customer

  ColorPaletteGenerator::DmColorMode(
      m_ColorPalette, m_nDepthColorMapMode, m_d8Far, m_d8Near);
  ColorPaletteGenerator::DmGrayMode(
      m_GrayPalette, m_nDepthColorMapMode, m_d8Far, m_d8Near);

  ColorPaletteGenerator::DmColorMode11(
      m_ColorPaletteD11, m_nDepthColorMapMode, m_d11Far, m_d11Near);
  ColorPaletteGenerator::DmGrayMode11(
      m_GrayPaletteD11, m_nDepthColorMapMode, m_d11Far, m_d11Near);
  // SetBaseGrayPaletteD11(m_GrayPaletteD11);

  ColorPaletteGenerator::DmColorMode14(m_ColorPaletteZ14, m_zFar, m_zNear);
  ColorPaletteGenerator::DmGrayMode14(m_GrayPaletteZ14, m_zFar, m_zNear);
  // SetBaseGrayPaletteZ14(m_GrayPaletteZ14, zFar);
}

bool Device::Restart() {
  EtronDI_CloseDevice(handle_, &dev_sel_info_);
  EtronDI_Release(&handle_);
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
