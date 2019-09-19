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
#include "color_palette_generator.h"

MYNTEYE_USE_NAMESPACE

namespace {

#define Z14_NEAR 0
#define D11_FAR  0
#define D11_NEAR 2047
#define D8_FAR   0
#define D8_NEAR  255

void generatePaletteColor(RGBQUAD* palette, int size, int mode, int customROI1, int customROI2, bool reverseRedToBlue) {
	float ROI2 = 1.0f;
	float ROI1 = 0.0f;

	//The value ranges from 0.0f ~ 1.0f as hue angle
	float ROI2Value = 1.0f;
	float ROI1Value = 0.0f;

	BYTE* buf = (BYTE*)malloc(sizeof(BYTE) * 4 * size);
	//BYTE buf[(size) * 4];
	//Set ROI by mode setting.The bigger the disparity the nearer the distance
	switch (mode) {
	case 1: //near
		ROI2 = 0.8f;
		ROI1 = 0.5f;
		ROI2Value = 0.9f;
		ROI1Value = 0.1f;
		break;
	case 2: //midle
		ROI2 = 0.7f;
		ROI1 = 0.3f;
		ROI2Value = 0.9f;
		ROI1Value = 0.1f;
		break;
	case 3: //far
		ROI2 = 0.6f;
		ROI1 = 0.2f;
		ROI2Value = 0.9f;
		ROI1Value = 0.1f;
		break;
	case 4: //custom
		ROI2 = 1.0f*customROI2 / size;
		ROI1 = 1.0f*customROI1 / size;
		ROI2Value = 1.0f;
		ROI1Value = 0.0f;
		break;
	default: //normal 
		ROI2 = 1.0f;
		ROI1 = 0.0f;
		ROI2Value = 1.0f;
		ROI1Value = 0.0f;
		break;
	}

	ColorPaletteGenerator::generatePalette(buf, size, ROI1 * size, ROI1Value, ROI2 * size, ROI2Value, reverseRedToBlue);
	for (int i = 0; i < size; i++) {
		palette[i].rgbBlue		= buf[i * 4 + 2];
		palette[i].rgbGreen		= buf[i * 4 + 1];
		palette[i].rgbRed		= buf[i * 4 + 0];
		palette[i].rgbReserved	= buf[i * 4 + 3];
	}
}


void generatePaletteGray(RGBQUAD* palette, int size, int mode, int customROI1, int customROI2, bool reverseGraylevel){
	float ROI1 = 0.0f;
	float ROI2 = 1.0f;
	

	//The value ranges from 0.0f ~ 1.0f as hue angle
	float ROI1Value = 0.0f;
	float ROI2Value = 1.0f;
	

	BYTE* buf = (BYTE*)malloc(sizeof(BYTE) * 4 * size);
	//BYTE buf[(size) * 4];
	//Set ROI by mode setting.The bigger the disparity the nearer the distance
	switch (mode) {
	case 1: //near
		ROI2 = 0.8f;
		ROI1 = 0.5f;
		ROI2Value = 0.9f;
		ROI1Value = 0.1f;
		break;
	case 2: //midle
		ROI2 = 0.7f;
		ROI1 = 0.3f;
		ROI2Value = 0.9f;
		ROI1Value = 0.1f;
		break;
	case 3: //far
		ROI2 = 0.6f;
		ROI1 = 0.2f;
		ROI2Value = 0.9f;
		ROI1Value = 0.1f;
		break;
	case 4: //custom
		ROI2 = 1.0f*customROI2 / size;
		ROI1 = 1.0f*customROI1 / size;
		ROI2Value = 1.0f;
		ROI1Value = 0.0f;
		break;
	default: //normal 
		ROI2 = 1.0f;
		ROI1 = 0.0f;
		ROI2Value = 1.0f;
		ROI1Value = 0.0f;
		break;
	}

	ColorPaletteGenerator::generatePaletteGray(buf, size, ROI1 * size, ROI1Value, ROI2 * size, ROI2Value, reverseGraylevel);
	for (int i = 0; i < size; i++) {
		palette[i].rgbBlue = buf[i * 4 + 2];
		palette[i].rgbGreen = buf[i * 4 + 1];
		palette[i].rgbRed = buf[i * 4 + 0];
		palette[i].rgbReserved = buf[i * 4 + 3];
	}
}

void DmColorMode(unsigned char palette[256][4], int mode,int d8Far, int d8Near)
{
		
	const int size = 1 << 8; // 8bits = 256
	bool reverseRedtoBlue = true;
	RGBQUAD RGBpalette[size];
	generatePaletteColor(RGBpalette, size, mode, d8Far, d8Near, reverseRedtoBlue);

	for (int i = 0; i < size; i++) {
		palette[i][0] = RGBpalette[i].rgbBlue;
		palette[i][1] = RGBpalette[i].rgbGreen;
		palette[i][2] = RGBpalette[i].rgbRed;
		palette[i][3] = RGBpalette[i].rgbReserved;
	}
}
#ifdef ENABLE_LONG_DEPTHCOLOR_MAP
void DmColorMode11(RGBQUAD palette[2048], int mode,int d11Far,int d11Near)
{
	const int size = 1 << 11; // 11 bits = 2048
	bool reverseRedtoBlue = true;
	generatePaletteColor(palette, size, mode, d11Far, d11Near, reverseRedtoBlue);
}

void DmGrayMode11(RGBQUAD palette[2048], int mode,int d11Far,int d11Near)
{

	const int size = 1 << 11; // 11 bits = 2048
	bool reverseGraylevel = false;
	generatePaletteGray(palette, size, mode, d11Far, d11Near, reverseGraylevel);
}
void DmColorMode14(RGBQUAD palette[16384],float zFar,float zNear)
{
	const int size = 1 << 14; // 14 bits  
	bool reverseRedtoBlue = false;
	int mode = 4; //custom
	generatePaletteColor(palette, size,mode, zNear, zFar, reverseRedtoBlue);
}

void DmGrayMode14(RGBQUAD *palette,  float zFar,float zNear)
{
	const int size = 1 << 14; // 14bit 16384
	int mode = 4;//custom
	bool reverseGraylevel = true;
	generatePaletteGray(palette, size, mode, zNear, zFar, reverseGraylevel);
}
#else//not ENABLE_LONG_DEPTHCOLOR_MAP
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

void HSV_to_RGB(double H, double S, double V, double &R, double &G, double &B) {
  double nMax, nMin;
  double fDet;
  //
  while (H < 0.0) H += 360.0;
  while (H >= 360.0) H -= 360.0;
  H /= 60.0;
  if (V < 0.0) V = 0.0;
  if (V > 1.0) V = 1.0;
  V *= 255.0;
  if (S < 0.0) S = 0.0;
  if (S > 1.0) S = 1.0;
  //
  if (V == 0.0) {
    R = G = B = 0;
  } else {
    fDet = S*V;
    nMax = (V);
    nMin = (V-fDet);
    if (H <= 1.0) { // R>=G>=B, H=(G-B)/fDet
      R = nMax;
      B = nMin;
      G = (H*fDet+B);
    } else if (H <= 2.0) { // G>=R>=B, H=2+(B-R)/fDet
      G = nMax;
      B = nMin;
      R = ((2.0-H)*fDet+B);
    } else if (H <= 3.0) { // G>=B>=R, H=2+(B-R)/fDet
      G = nMax;
      R = nMin;
      B = ((H-2.0)*fDet+R);
    } else if (H <= 4.0) { // B>=G>=R, H=4+(R-G)/fDet
      B = nMax;
      R = nMin;
      G = ((4.0-H)*fDet+R);
    } else if (H <= 5.0) { // B>=R>=G, H=4+(R-G)/fDet
      B = nMax;
      G = nMin;
      R = ((H-4.0)*fDet+G);
    } else { // if(H<6.0) // R>=B>=G, H=(G-B)/fDet+6
      R = nMax;
      G = nMin;
      B = ((6.0-H)*fDet+G);
    }
  }
}

void SetBaseGrayPaletteZ14(RGBQUAD *pGrayPaletteZ14) {
  int i;
  double R, G, B;
  double fx, fy;
  //
  double fCV = 180;
  int nCenter = 1500;
  double r1 = 0.35;
  double r2 = 0.55;
  //
  for (i = 1; i < 16384; i++) {
    if (i == nCenter) {
      fy = fCV;
    } else if (i < nCenter) {
      fx = (double)(nCenter - i) / nCenter;
      fy = fCV - pow(fx, r1)*fCV;
    } else {
      fx = (double)(i - nCenter) / (16384 - nCenter);
      fy = fCV + pow(fx, r2)*(256-fCV);
    }
    HSV_to_RGB(fy, 1.0, 1.0, R, G, B);
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

void DmColorMode11(RGBQUAD pallete[2048], int mode)
{
	double R, G, B;
	for (int i = 0; i<2048; i++) {
		HSV_to_RGB((double)(i >> 3), 1.0, 1.0, R, G, B);

		pallete[i].rgbBlue = (BYTE)B;
		pallete[i].rgbGreen = (BYTE)G;
		pallete[i].rgbRed = (BYTE)R;
		pallete[i].rgbReserved = 0;
	}
}
void DmColorMode14(RGBQUAD *pColorPaletteZ14, int mode)
{
	int i;
	double R, G, B;
	double fx, fy;
	//
	double fCV = 180;
	int nCenter = 1500;
	double r1 = 0.35;
	double r2 = 0.55;
	//
	for (i = 1; i<16384; i++) {
		if (i == nCenter) {
			fy = fCV;
		}
		else if (i<nCenter) {
			fx = (double)(nCenter - i) / nCenter;
			fy = fCV - pow(fx, r1)*fCV;
		}
		else {
			fx = (double)(i - nCenter) / (16384 - nCenter);
			fy = fCV + pow(fx, r2)*(256 - fCV);
		}
		HSV_to_RGB(fy, 1.0, 1.0, R, G, B);
		pColorPaletteZ14[i].rgbBlue = (BYTE)B;
		pColorPaletteZ14[i].rgbGreen = (BYTE)G;
		pColorPaletteZ14[i].rgbRed = (BYTE)R;
		pColorPaletteZ14[i].rgbReserved = 0;
	}
	{
		i = 0;
		pColorPaletteZ14[i].rgbBlue = (BYTE)0;
		pColorPaletteZ14[i].rgbGreen = (BYTE)0;
		pColorPaletteZ14[i].rgbRed = (BYTE)0;
		pColorPaletteZ14[i].rgbReserved = 0;
	}
	{
		i = 16383;
		pColorPaletteZ14[i].rgbBlue = (BYTE)255;
		pColorPaletteZ14[i].rgbGreen = (BYTE)255;
		pColorPaletteZ14[i].rgbRed = (BYTE)255;
		pColorPaletteZ14[i].rgbReserved = 0;
	}
}
#endif //ENABLE_LONG_DEPTHCOLOR_MAP

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
  };
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
  };
  is_depth_ok_ = false;

  if (depth_image_buf_) {
    device_status_[DEPTH_DEVICE] = true;
    is_actual_[DEPTH_DEVICE] = true;

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
        UpdateZ14DisplayImage_DIB24(m_GrayPaletteZ14,
            depth_image_buf_->data(), depth_gray_buf->data(),
            depth_img_width, depth_img_height);
        return depth_gray_buf;
      } break;
      case DepthMode::DEPTH_COLORFUL: {
        static auto depth_rgb_buf = ImageDepth::Create(ImageFormat::DEPTH_RGB,
            depth_img_width, depth_img_height, true);
        depth_rgb_buf->ResetBuffer();
        depth_rgb_buf->set_frame_id(depth_image_buf_->frame_id());
        UpdateZ14DisplayImage_DIB24(m_ColorPaletteZ14,
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

void Device::OnInitColorPalette(const float &z14_Far) {
  float m_zFar = z14_Far;
  float m_zNear = Z14_NEAR;
  int m_d11Far = D11_FAR;
  int m_d11Near = D11_NEAR;
  int m_d8Far = D8_FAR;
  int m_d8Near = D8_NEAR;

  int m_nDepthColorMapMode = 0;

  DmColorMode(m_ColorPalette, m_nDepthColorMapMode, m_d8Far, m_d8Near);
  DmColorMode11(m_ColorPaletteD11, m_nDepthColorMapMode, m_d11Far, m_d11Near);
	DmGrayMode11(m_GrayPaletteD11, m_nDepthColorMapMode, m_d11Far, m_d11Near);
  DmColorMode14(m_ColorPaletteZ14, m_zFar, m_zNear);
  DmGrayMode14(m_GrayPaletteZ14, m_zFar, m_zNear);
  // ToDo
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
