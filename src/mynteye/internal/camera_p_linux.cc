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

#ifdef MYNTEYE_OS_LINUX

#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/util/convertor.h"
#include "mynteye/util/log.h"

MYNTEYE_USE_NAMESPACE

// int ret = EtronDI_Get2Image(etron_di_, &dev_sel_info_,
//     (BYTE*)color_img_buf_, (BYTE*)depth_img_buf_,
//     &color_image_size_, &depth_image_size_,
//     &color_serial_number_, &depth_serial_number_, depth_data_type_);

ErrorCode CameraPrivate::RetrieveImageColor(cv::Mat* color_) {
  unsigned int color_img_width  = (unsigned int)(
      stream_color_info_ptr_[color_res_index_].nWidth);
  unsigned int color_img_height = (unsigned int)(
      stream_color_info_ptr_[color_res_index_].nHeight);
  bool is_mjpeg = stream_color_info_ptr_[color_res_index_].bFormatMJPG;

  if (!color_img_buf_) {
    color_img_buf_ = (unsigned char*)calloc(
        color_img_width*color_img_height*2, sizeof(unsigned char));
  }
  if (is_mjpeg && !color_rgb_buf_) {
    color_rgb_buf_ = (unsigned char*)calloc(
        color_img_width*color_img_height*3, sizeof(unsigned char));
  }

  int ret = EtronDI_GetColorImage(etron_di_, &dev_sel_info_,
      color_img_buf_, &color_image_size_, &color_serial_number_, 0);

  if (ETronDI_OK != ret) {
    DBG_LOGI("EtronDI_GetColorImage: %d", ret);
    return ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
  }

  cv::Mat& color = *color_;

  if (is_mjpeg) {
    MJPEG_TO_RGB24_LIBJPEG(color_img_buf_, color_image_size_, color_rgb_buf_);
    cv::Mat color_img(color_img_height, color_img_width, CV_8UC3,
        color_rgb_buf_);
    cv::cvtColor(color_img, color, CV_RGB2BGR);
  } else {
    cv::Mat color_img(color_img_height, color_img_width, CV_8UC2,
        color_img_buf_);
    cv::cvtColor(color_img, color, CV_YUV2BGR_YUY2);
  }

  return ErrorCode::SUCCESS;
}

ErrorCode CameraPrivate::RetrieveImageDepth(cv::Mat* depth_) {
  unsigned int depth_img_width  = (unsigned int)(
      stream_depth_info_ptr_[depth_res_index_].nWidth);
  unsigned int depth_img_height = (unsigned int)(
      stream_depth_info_ptr_[depth_res_index_].nHeight);

  if (!depth_img_buf_) {
    if (dtc_ == DEPTH_IMG_COLORFUL_TRANSFER ||
        dtc_ == DEPTH_IMG_GRAY_TRANSFER) {
      depth_img_buf_ = (unsigned char*)calloc(
          depth_img_width*2*depth_img_height*3, sizeof(unsigned char));
    } else {
      depth_img_buf_ = (unsigned char*)calloc(
          depth_img_width*depth_img_height*2, sizeof(unsigned char));
    }
  }

  int ret = EtronDI_GetDepthImage(etron_di_, &dev_sel_info_,
      depth_img_buf_, &depth_image_size_,
      &depth_serial_number_, depth_data_type_);

  if (ETronDI_OK != ret) {
    DBG_LOGI("EtronDI_GetColorImage: %d", ret);
    return ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
  }

  cv::Mat& depth = *depth_;

  if (dtc_ == DEPTH_IMG_COLORFUL_TRANSFER || dtc_ == DEPTH_IMG_GRAY_TRANSFER) {
    // Depth data type: 11 bits & 14 bits
    cv::Mat depth_img(depth_img_height, depth_img_width, CV_8UC3,
        depth_img_buf_);
    cv::cvtColor(depth_img, depth, CV_RGB2BGR);
  } else {  // DEPTH_IMG_NON_TRANSFER
    cv::Mat depth_img(depth_img_height, depth_img_width, CV_8UC2,
        depth_img_buf_);
    if (depth_mode_ == DepthMode::DEPTH_NON) {
      cv::cvtColor(depth_img, depth, CV_YUV2BGR_YUY2);
    } else {
      const int h = static_cast<int>(depth_img_height);
      const int w = static_cast<int>(depth_img_width);
      if (depth_raw_.rows != h || depth_raw_.cols != w) {
        depth_raw_ = cv::Mat(h, w, CV_16UC1);
      }
      // initialize depth min,max
      const cv::Vec2b& pixel = depth_img.at<cv::Vec2b>(0, 0);
      ushort depth_pixel = (pixel[0] & 0xff) | ((pixel[1] & 0xff) << 8);
      depth_min_ = depth_max_ = depth_pixel;
      // compute depth pixels
      for (int i = 0; i < h; ++i) {  // row
        for (int j = 0; j < w; ++j) {  // col
          const cv::Vec2b& pixel = depth_img.at<cv::Vec2b>(i, j);
          depth_pixel = (pixel[0] & 0xff) | ((pixel[1] & 0xff) << 8);
          depth_raw_.at<ushort>(i, j) = depth_pixel;
          if (depth_pixel < depth_min_) depth_min_ = depth_pixel;
          if (depth_pixel > depth_max_) depth_max_ = depth_pixel;
        }
      }

      if (depth_mode_ == DepthMode::DEPTH_NON_16UC1) {
        depth = depth_raw_;
      } else if (depth_mode_ == DepthMode::DEPTH_NON_8UC1) {
        if (depth.rows != h || depth.cols != w || depth.type() != CV_8UC1) {
          depth = cv::Mat(h, w, CV_8UC1);
        }
        // transfer depth to gray
        ushort depth_dist = depth_max_ - depth_min_;
        for (int i = 0; i < h; ++i) {  // row
          for (int j = 0; j < w; ++j) {  // col
            const ushort depth_pixel = depth_raw_.at<ushort>(i, j);
            depth.at<uchar>(i, j) =
                255 * (depth_pixel - depth_min_) / depth_dist;
          }
        }
      } else {
        throw new std::runtime_error("Error: Depth mode is not supported.");
      }
    }
  }

  return ErrorCode::SUCCESS;
}

#endif
