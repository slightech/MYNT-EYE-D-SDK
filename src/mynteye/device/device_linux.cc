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
#include "mynteye/device/device.h"

#ifdef MYNTEYE_OS_LINUX

#include <algorithm>

#include "mynteye/device/convertor.h"
#include "mynteye/util/log.h"

MYNTEYE_USE_NAMESPACE

void Device::OnInit() {
  dtc_ = DEPTH_IMG_NON_TRANSFER;
}

// int ret = EtronDI_Get2Image(etron_di_, &dev_sel_info_,
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

  int ret = EtronDI_GetColorImage(etron_di_, &dev_sel_info_,
      color_image_buf_->data(), &color_image_size_, &color_serial_number_, 0);

  if (ETronDI_OK != ret) {
    DBG_LOGI("GetImageColor: %d", ret);
    return nullptr;
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
    if (!depth_image_buf_) {
      depth_buf_ = (unsigned char*)calloc(
          depth_img_width * 2 * depth_img_height * 3, sizeof(unsigned char));
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

  int ret = EtronDI_GetDepthImage(etron_di_, &dev_sel_info_,
      depth_raw ? depth_image_buf_->data() : depth_buf_,
      &depth_image_size_, &depth_serial_number_, depth_data_type_);

  if (ETronDI_OK != ret) {
    DBG_LOGI("GetImageDepth: %d", ret);
    return nullptr;
  }

  depth_image_buf_->set_frame_id(depth_serial_number_);

  if (depth_raw) {
    return depth_image_buf_;
  } else {
    depth_image_buf_->set_valid_size(depth_image_size_);

    std::copy(depth_buf_, depth_buf_ + depth_image_size_,
        depth_image_buf_->data());
    // EtronDI_Convert_Depth_Y_To_Buffer(etron_di_, &dev_sel_info_,
    //   depth_buf_, depth_image_buf_->data(),
    //   depth_img_width, depth_img_height,
    //   dtc_ == DEPTH_IMG_COLORFUL_TRANSFER ? true : false,
    //   depth_data_type_);
    return depth_image_buf_;
  }
}

#endif
