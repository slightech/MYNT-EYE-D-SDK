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

#ifdef MYNTEYE_OS_WIN

#include "mynteye/util/convertor.h"
#include "mynteye/util/log.h"

MYNTEYE_USE_NAMESPACE

namespace {

// k= 0~1.00
// maps k to a pixel color RGB
void ColorMap(double k, double& R, double& G, double& B) {  // NOLINT
  double r;

  if (k < 0.0) k = 0.0;
  if (k > 1.0) k = 1.0;
  if (k < 0.1) {
    r = k / 0.1;
    R = G = B = 128.0 + r * 127.0;  // 128~255
  } else if (k < 0.2) {
    k -= .1;
    r = k / 0.1;
    R = 255.0;
    G = B = (1.0 - r) * 255.0;  // 255~0
  } else if (k < 0.35) {
    k -= .2;
    r = k / 0.15;
    B = 0.0;  // B
    G = r * 255.0;  // 0~255
    R = 255.0;  // R
  } else if (k < 0.5) {
    k -= 0.35;
    r = k / 0.15;
    B = 0.0;
    G = (1.0 - r / 4.0) * 255.0;  // 255~196
    R = (1.0 - r / 2.0) * 255.0;  // 255~128
  } else if (k < 0.6) {
    k -= 0.5;
    r = k / 0.1;
    B = r * 128.0;  // B 0~128
    G = 196.0;  // G
    R = (1.0 - r) * 128.0;  // R 128~0
  } else if (k < 0.7) {
    k -= 0.6;
    r = k / 0.1;
    B = 128.0 + r * 127.0;  // B 128~255
    G = 196.0;  // G
    R = 0.0;  // R
  } else if (k < 0.8) {
    k -= 0.7;
    r = k / 0.1;
    B = 255;  // B
    G = (1.0 - r) * 196.0;  // G 196~0
    R = 0;  // R
  } else if (k < 0.9) {
    k -= 0.8;
    r = k / 0.1;
    B = (1.0 - r / 2.0) * 255.0;  // B 255~128
    G = 0.0;  // G
    R = r * 128.0;  // R=0~128
  } else {
    k -= .9;
    r = k / .1;
    R = B = (1 - r) * 128;  // B 128~0
    G = 0;  // G
  }
}

// ENABLE_LONG_DEPTHCOLOR_MAP
void DmColorMode14(RGBQUAD* pallete, int mode = 0) {
#define CP1 0.75
#define CP2 0.25
  int length = 16384;
  int i;
  double R, G, B;
  int t1, t2;  // focus region, 0.25~0.75 mapping area
  switch (mode) {
  case 1:  // near
    t1 = 512*8;
    t2 = 1024 * 8;
    break;
  case 2:  // midle
    t1 = 200 * 8;
    t2 = 512 * 8;
    break;
  case 3:  // far
    t1 = 5 * 8;
    t2 = 256 * 8;
    break;
  default:  // normal
    t1 = 256 * 8;
    t2 = 512 * 8;
    break;
  }
  double m, b;  // y=mx+b
  m = (CP1 - 1.0) / (double)t1;  // NOLINT
  b = 1.0;
  for (i = 0; i < t1; i++) {
    ColorMap(m * (double)i + b, R, G, B);  // NOLINT
    pallete[i].rgbBlue = (BYTE)B;
    pallete[i].rgbGreen = (BYTE)G;
    pallete[i].rgbRed = (BYTE)R;
    pallete[i].rgbReserved = 0;
  }
  m = (CP2 - CP1) / (double)(t2 - t1);  // NOLINT
  b = CP1 - m * (double)t1;  // NOLINT
  for (; i < t2; i++) {
    ColorMap(m * (double)i + b, R, G, B);  // NOLINT
    pallete[i].rgbBlue = (BYTE)B;
    pallete[i].rgbGreen = (BYTE)G;
    pallete[i].rgbRed = (BYTE)R;
    pallete[i].rgbReserved = 0;
  }
  m = (0 - CP2) / (double)(2048 - t2);  // NOLINT
  b = CP2 - m * (double)t2;  // NOLINT
  for (; i < length; i++) {
    ColorMap(m * (double)i + b, R, G, B);  // NOLINT
    pallete[i].rgbBlue = (BYTE)B;
    pallete[i].rgbGreen = (BYTE)G;
    pallete[i].rgbRed = (BYTE)R;
    pallete[i].rgbReserved = 0;
  }
}

void UpdateZ14DisplayImage_DIB24(RGBQUAD* pColorPaletteZ14, BYTE* pDepthZ14,
    BYTE* pDepthDIB24, int cx, int cy) {
  int x, y, nBPS;
  WORD *pWSL, *pWS;
  BYTE *pDL, *pD;
  RGBQUAD *pClr;

  if ((cx <= 0) || (cy <= 0)) return;

  nBPS = ((cx*3+3)/4)*4;
  pWSL = (WORD*)pDepthZ14;  // NOLINT
  // pDL = pDepthDIB24 + (cy-1)*nBPS;
  pDL = pDepthDIB24;
  for (y = 0; y < cy; y++) {
    pWS = pWSL;
    pD = pDL;
    for (x = 0; x < cx; x++) {
      pClr = &(pColorPaletteZ14[pWS[x]]);
      pD[0] = pClr->rgbBlue;  // B
      pD[1] = pClr->rgbGreen;  // G
      pD[2] = pClr->rgbRed;  // R
      pD += 3;
    }
    pWSL += cx;
    // pDL -= nBPS;
    pDL += nBPS;
  }
}

}  // namespace

void CameraPrivate::OnInit() {
  DmColorMode14(color_palette_z14_, 0/*normal*/);
}

void CameraPrivate::ImgCallback(EtronDIImageType::Value imgType, int imgId,
      unsigned char* imgBuf, int imgSize, int width, int height,
      int serialNumber, void *pParam) {
  CameraPrivate* p = static_cast<CameraPrivate*>(pParam);
  std::lock_guard<std::mutex> _(p->mtx_imgs_);

  if (EtronDIImageType::IsImageColor(imgType)) {
    // LOGI("Image callback color");
    if (!p->color_img_buf_) {
      unsigned int color_img_width  =(unsigned int)(
          p->stream_color_info_ptr_[p->color_res_index_].nWidth);
      unsigned int color_img_height =(unsigned int)(
          p->stream_color_info_ptr_[p->color_res_index_].nHeight);
      p->color_img_buf_ = (unsigned char*)calloc(
          color_img_width*color_img_height*3, sizeof(unsigned char));
    }

    memcpy(p->color_img_buf_, imgBuf, imgSize);

    p->is_color_rgb24_ = (imgType == EtronDIImageType::COLOR_RGB24);
    p->is_color_mjpg_ = (imgType == EtronDIImageType::COLOR_MJPG);
  } else if (EtronDIImageType::IsImageDepth(imgType)) {
    // LOGI("Image callback depth");
    if (!p->depth_img_buf_) {
      unsigned int depth_img_width  = (unsigned int)(
          p->stream_depth_info_ptr_[p->depth_res_index_].nWidth);
      unsigned int depth_img_height = (unsigned int)(
          p->stream_depth_info_ptr_[p->depth_res_index_].nHeight);
      p->depth_img_buf_ = (unsigned char*)calloc(
          depth_img_width*depth_img_height*2, sizeof(unsigned char));
      p->depth_data_size_ = depth_img_width*depth_img_height*2;
    }
    memcpy(p->depth_img_buf_, imgBuf, p->depth_data_size_);
  } else {
    LOGE("Image callback failed. Unknown image type.");
  }
}

ErrorCode CameraPrivate::RetrieveImageColor(cv::Mat* color_) {
  // LOGI("Retrieve image color");
  if (!color_img_buf_) {
    return ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
  }
  std::lock_guard<std::mutex> _(mtx_imgs_);

  cv::Mat& color = *color_;

  bool color_ok = false;
  if (color_img_buf_) {
    unsigned int color_img_width  = (unsigned int)(
        stream_color_info_ptr_[color_res_index_].nWidth);
    unsigned int color_img_height = (unsigned int)(
        stream_color_info_ptr_[color_res_index_].nHeight);
    if (is_color_mjpg_) {  // mjpg
      if (!color_rgb_buf_) {
        color_rgb_buf_ = (unsigned char*)calloc(
            color_img_width*color_img_height*3, sizeof(unsigned char));
      }
      MJPEG_TO_RGB24_LIBJPEG(color_img_buf_, color_image_size_, color_rgb_buf_);
      cv::Mat color_img(color_img_height, color_img_width, CV_8UC3,
          color_rgb_buf_);
      cv::cvtColor(color_img, color, CV_RGB2BGR);
      color_ok = true;
    } else if (is_color_rgb24_) {  // rgb24
      cv::Mat color_img(color_img_height, color_img_width, CV_8UC3,
          color_img_buf_);
      cv::flip(color_img, color, 0);
      color_ok = true;
    } else {
      LOGE("Unknown image color type.");
    }
  }

  return color_ok ? ErrorCode::SUCCESS :
     ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
}

ErrorCode CameraPrivate::RetrieveImageDepth(cv::Mat* depth_) {
  // LOGI("Retrieve image depth");
  if (!depth_img_buf_) {
    return ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
  }
  std::lock_guard<std::mutex> _(mtx_imgs_);

  cv::Mat& depth = *depth_;

  bool depth_ok = false;
  if (depth_img_buf_) {
    // DEPTH_14BITS for ETronDI_DEPTH_DATA_14_BITS
    unsigned int depth_img_width  = (unsigned int)(
        stream_depth_info_ptr_[depth_res_index_].nWidth);
    unsigned int depth_img_height = (unsigned int)(
        stream_depth_info_ptr_[depth_res_index_].nHeight);

    depth_raw_ = cv::Mat(depth_img_height, depth_img_width, CV_16UC1,
        depth_img_buf_);

    switch (depth_mode_) {
      case DepthMode::DEPTH_GRAY:
      case DepthMode::DEPTH_NON_8UC1:
        cv::normalize(depth_raw_, depth, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        break;
      case DepthMode::DEPTH_COLORFUL:
        if (!depth_rgb_buf_) {
          depth_rgb_buf_ = (unsigned char*)calloc(
              depth_img_width*depth_img_height*3, sizeof(unsigned char));
        }
        UpdateZ14DisplayImage_DIB24(color_palette_z14_, depth_img_buf_,
            depth_rgb_buf_, depth_img_width, depth_img_height);
        depth = cv::Mat(depth_img_height, depth_img_width, CV_8UC3,
            depth_rgb_buf_);
        break;
      case DepthMode::DEPTH_NON:
      case DepthMode::DEPTH_NON_16UC1:
      default:
        depth = depth_raw_;
        break;
    }

    depth_ok = true;

    // test the depth value: 2 bytes, unshort

    /*
    const int h = static_cast<int>(depth_img_height);
    const int w = static_cast<int>(depth_img_width);
    if (depth_raw_.rows != h || depth_raw_.cols != w) {
      depth_raw_ = cv::Mat(h, w, CV_16UC1);
    }
    for (int i = 0; i < h; ++i) {  // row
      for (int j = 0; j < w; ++j) {  // col
        depth_raw_.at<ushort>(i,j) = *(WORD*)(&depth_img_buf_[(i*w+j) * sizeof(WORD)]);
      }
    }
    */

    /*
    cv::Mat depth_img(depth_img_height, depth_img_width, CV_8UC2, depth_img_buf_);

    const int h = static_cast<int>(depth_img_height);
    const int w = static_cast<int>(depth_img_width);
    if (depth_raw_.rows != h || depth_raw_.cols != w) {
      depth_raw_ = cv::Mat(h, w, CV_16UC1);
    }
    // initialize depth min,max
    const cv::Vec2b& pixel = depth_img.at<cv::Vec2b>(0,0);
    ushort depth_pixel = (pixel[0] & 0xff) | ((pixel[1] & 0xff) << 8);
    depth_min_ = depth_max_ = depth_pixel;
    // compute depth pixels
    for (int i = 0; i < h; ++i) {  // row
      for (int j = 0; j < w; ++j) {  // col
        const cv::Vec2b& pixel = depth_img.at<cv::Vec2b>(i,j);
        depth_pixel = (pixel[0] & 0xff) | ((pixel[1] & 0xff) << 8);
        depth_raw_.at<ushort>(i,j) = depth_pixel;
        if (depth_pixel < depth_min_) depth_min_ = depth_pixel;
        if (depth_pixel > depth_max_) depth_max_ = depth_pixel;
      }
    }
    */
  }

  return depth_ok ? ErrorCode::SUCCESS :
     ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
}

#endif
