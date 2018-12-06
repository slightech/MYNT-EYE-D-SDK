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

void HSV_to_RGB(double H, double S, double V, double &R, double &G, double &B) {
  double nMax,nMin;
  double fDet;
  //
  while (H<0.0) H+=360.0;
  while (H>=360.0) H-=360.0;
  H /= 60.0;
  if (V<0.0) V = 0.0;
  if (V>1.0) V = 1.0;
  V *= 255.0;
  if (S<0.0) S = 0.0;
  if (S>1.0) S = 1.0;
  //
  if (V == 0.0) {
    R = G = B = 0;
  } else {
    fDet = S*V;
    nMax = (V);
    nMin = (V-fDet);
    if (H<=1.0) { //R>=G>=B, H=(G-B)/fDet
      R = nMax;
      B = nMin;
      G = (H*fDet+B);
    } else if (H<=2.0) { //G>=R>=B, H=2+(B-R)/fDet
      G = nMax;
      B = nMin;
      R = ((2.0-H)*fDet+B);
    } else if (H<=3.0) { //G>=B>=R, H=2+(B-R)/fDet
      G = nMax;
      R = nMin;
      B = ((H-2.0)*fDet+R);
    } else if (H<=4.0) { //B>=G>=R, H=4+(R-G)/fDet
      B = nMax;
      R = nMin;
      G = ((4.0-H)*fDet+R);
    } else if (H<=5.0) { //B>=R>=G, H=4+(R-G)/fDet
      B = nMax;
      G = nMin;
      R = ((H-4.0)*fDet+G);
    } else { // if(H<6.0) //R>=B>=G, H=(G-B)/fDet+6
      R = nMax;
      G = nMin;
      B = ((6.0-H)*fDet+G);
    }
  }
}

void SetBaseGrayPaletteZ14(RGBQUAD *pGrayPaletteZ14) {
  int i;
  double R,G,B;
  double fx,fy;
  //
  double fCV = 180;
  int nCenter=1500;
  double r1=0.35;
  double r2=0.55;
  //
  for (i=1; i<16384; i++) {
    if (i==nCenter) {
      fy = fCV;
    } else if (i<nCenter) {
      fx = (double)(nCenter-i)/nCenter;
      fy = fCV - pow(fx, r1)*fCV;
    } else {
      fx = (double)(i-nCenter)/(16384-nCenter);
      fy = fCV + pow(fx, r2)*(256-fCV);
    }
    HSV_to_RGB(fy,1.0,1.0,R,G,B);
    pGrayPaletteZ14[i].rgbBlue     = (BYTE)B;
    pGrayPaletteZ14[i].rgbGreen    = (BYTE)B;
    pGrayPaletteZ14[i].rgbRed      = (BYTE)B;
    pGrayPaletteZ14[i].rgbReserved = 0;
  }
  {
    i = 0;
    pGrayPaletteZ14[i].rgbBlue     = (BYTE)0;
    pGrayPaletteZ14[i].rgbGreen    = (BYTE)0;
    pGrayPaletteZ14[i].rgbRed      = (BYTE)0;
    pGrayPaletteZ14[i].rgbReserved = 0;
  }
  {
    i = 16383;
    pGrayPaletteZ14[i].rgbBlue     = (BYTE)255;
    pGrayPaletteZ14[i].rgbGreen    = (BYTE)255;
    pGrayPaletteZ14[i].rgbRed      = (BYTE)255;
    pGrayPaletteZ14[i].rgbReserved = 0;
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

void Device::OnInit() {
  is_color_ok_ = false;
  is_depth_ok_ = false;
  DmColorMode14(color_palette_z14_, 3/*far*/);
  SetBaseGrayPaletteZ14(gray_palette_z14_);
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
  color_condition_.wait(lock, [this] { return is_color_ok_; });
  is_color_ok_ = false;

  if (color_image_buf_) {
    // unsigned int color_img_width  = (unsigned int)(
    //     stream_color_info_ptr_[color_res_index_].nWidth);
    // unsigned int color_img_height = (unsigned int)(
    //     stream_color_info_ptr_[color_res_index_].nHeight);

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
  depth_condition_.wait(lock, [this] { return is_depth_ok_; });
  is_depth_ok_ = false;

  if (depth_image_buf_) {
    // DEPTH_14BITS for ETronDI_DEPTH_DATA_14_BITS
    unsigned int depth_img_width  = (unsigned int)(
        stream_depth_info_ptr_[depth_res_index_].nWidth);
    unsigned int depth_img_height = (unsigned int)(
        stream_depth_info_ptr_[depth_res_index_].nHeight);

    switch (depth_mode_) {
      case DepthMode::DEPTH_RAW:
        // return clone as it will be changed in imgcallback
        return depth_image_buf_->Clone();
      case DepthMode::DEPTH_GRAY: {
        static auto depth_gray_buf = ImageDepth::Create(
            ImageFormat::DEPTH_GRAY_24,
            depth_img_width, depth_img_height, true);
        depth_gray_buf->ResetBuffer();
        depth_gray_buf->set_frame_id(depth_image_buf_->frame_id());
        UpdateZ14DisplayImage_DIB24(gray_palette_z14_,
            depth_image_buf_->data(), depth_gray_buf->data(),
            depth_img_width, depth_img_height);
        return depth_gray_buf;
      } break;
      case DepthMode::DEPTH_COLORFUL: {
        static auto depth_rgb_buf = ImageDepth::Create(ImageFormat::DEPTH_RGB,
            depth_img_width, depth_img_height, true);
        depth_rgb_buf->ResetBuffer();
        depth_rgb_buf->set_frame_id(depth_image_buf_->frame_id());
        UpdateZ14DisplayImage_DIB24(color_palette_z14_,
            depth_image_buf_->data(), depth_rgb_buf->data(),
            depth_img_width, depth_img_height);
        return depth_rgb_buf;
      } break;
    }
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

      return EtronDI_OpenDeviceEx(etron_di_, &dev_sel_info_,
        color_res_index_, toRgb,
        -1, depthStreamSwitch,
        Device::ImgCallback, this, &framerate_, ctrlMode);
    case DeviceMode::DEVICE_DEPTH:
      color_device_opened_ = false;
      depth_device_opened_ = true;

      return EtronDI_OpenDeviceEx(etron_di_, &dev_sel_info_,
        -1, toRgb,
        depth_res_index_, depthStreamSwitch,
        Device::ImgCallback, this, &framerate_, ctrlMode);
      break;
    case DeviceMode::DEVICE_ALL:
      color_device_opened_ = true;
      depth_device_opened_ = true;

      return EtronDI_OpenDeviceEx(etron_di_, &dev_sel_info_,
        color_res_index_, toRgb,
        depth_res_index_, depthStreamSwitch,
        Device::ImgCallback, this, &framerate_, ctrlMode);
    default:
      throw_error("ERROR:: DeviceMode is unknown.");
  }
}

#endif
