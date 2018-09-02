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
#include "camera_p.h"

#include <stdexcept>

#include <opencv2/imgproc/imgproc.hpp>

#include "log.hpp"

using namespace mynteye;

namespace {

METHODDEF(void)
my_error_exit(j_common_ptr cinfo) {
  /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
  my_error_ptr myerr = (my_error_ptr) cinfo->err;

  /* Always display the message. */
  /* We could postpone this until after returning, if we chose. */
  (*cinfo->err->output_message) (cinfo);

  /* Return control to the setjmp point */
  longjmp(myerr->setjmp_buffer, 1);
}

#ifdef OS_WIN

//k= 0~1.00
//maps k to a pixel color RGB
void ColorMap(double k, double& R, double& G, double& B) {
    double r;

    if (k < 0.0) k = 0.0;
    if (k > 1.0) k = 1.0;
    if (k < 0.1) {
        r = k / 0.1;
        R = G = B = 128.0 + r * 127.0; //128~255
    } else if (k < 0.2) {
        k -= .1;
        r = k / 0.1;
        R = 255.0;
        G = B = (1.0 - r) * 255.0;//255~0
    } else if (k < 0.35) {
        k -= .2;
        r = k / 0.15;
        B = 0.0; //B
        G = r * 255.0; // 0~255
        R = 255.0; //R
    } else if (k < 0.5) {
        k -= 0.35;
        r = k / 0.15;
        B = 0.0;
        G = (1.0 - r / 4.0) * 255.0;  //255~196
        R = (1.0 - r / 2.0) * 255.0; //255~128
    } else if (k < 0.6) {
        k -= 0.5;
        r = k / 0.1;
        B = r * 128.0; //B 0~128
        G = 196.0; //Gc
        R = (1.0 - r) * 128.0; //R 128~0
    } else if (k < 0.7) {
        k -= 0.6;
        r = k / 0.1;
        B = 128.0 + r * 127.0; //B 128~255
        G = 196.0; //G
        R = 0.0; //R
    } else if (k < 0.8) {
        k -= 0.7;
        r = k / 0.1;
        B = 255; //B
        G = (1.0 - r) * 196.0; //G 196~0
        R = 0; //R
    } else if (k < 0.9) {
        k -= 0.8;
        r = k / 0.1;
        B = (1.0 - r / 2.0) * 255.0; //B 255~128
        G = 0.0; //G
        R = r * 128.0; //R=0~128
    } else {
        k -= .9;
        r = k / .1;
        R = B = (1 - r) * 128; //B 128~0
        G = 0; //G
    }
}

// ENABLE_LONG_DEPTHCOLOR_MAP
void DmColorMode14(RGBQUAD *pallete, int mode = 0) {
#define CP1 0.75
#define CP2 0.25
    int length = 16384;
    int i;
    double R, G, B;
    int t1, t2; //focus region, 0.25~0.75 mapping area
    switch (mode) {
    case 1: //near
        t1 = 512*8;
        t2 = 1024 * 8;
        break;
    case 2: //midle
        t1 = 200 * 8;
        t2 = 512 * 8;
        break;
    case 3: //far
        t1 = 5 * 8;
        t2 = 256 * 8;
        break;
    default: //normal
        t1 = 256 * 8;
        t2 = 512 * 8;
        break;
    }
    double m, b; //y=mx+b
                 //0~t1
    m = (CP1 - 1.0) / (double)t1;
    b = 1.0;
    for (i = 0; i<t1; i++) {
        ColorMap(m* (double)i + b, R, G, B);
        pallete[i].rgbBlue = (BYTE)B;
        pallete[i].rgbGreen = (BYTE)G;
        pallete[i].rgbRed = (BYTE)R;
        pallete[i].rgbReserved = 0;
    }
    m = (CP2 - CP1) / (double)(t2 - t1);
    b = CP1 - m*(double)t1;
    for (; i<t2; i++) {
        ColorMap(m* (double)i + b, R, G, B);
        pallete[i].rgbBlue = (BYTE)B;
        pallete[i].rgbGreen = (BYTE)G;
        pallete[i].rgbRed = (BYTE)R;
        pallete[i].rgbReserved = 0;
    }
    m = (0 - CP2) / (double)(2048 - t2);
    b = CP2 - m*(double)t2;
    for (; i<length; i++) {
        ColorMap(m* (double)i + b, R, G, B);
        pallete[i].rgbBlue = (BYTE)B;
        pallete[i].rgbGreen = (BYTE)G;
        pallete[i].rgbRed = (BYTE)R;
        pallete[i].rgbReserved = 0;
    }
}

void UpdateZ14DisplayImage_DIB24(RGBQUAD *pColorPaletteZ14, BYTE *pDepthZ14, BYTE *pDepthDIB24, int cx, int cy) {
    int x,y,nBPS;
    WORD *pWSL,*pWS;
    BYTE *pDL,*pD;
    RGBQUAD *pClr;

    if ((cx<=0) || (cy<=0)) return;

    nBPS = ((cx*3+3)/4)*4;
    pWSL = (WORD*)pDepthZ14;
    // pDL = pDepthDIB24 + (cy-1)*nBPS;
    pDL = pDepthDIB24;
    for (y=0; y<cy; y++) {
        pWS = pWSL;
        pD = pDL;
        for (x=0; x<cx; x++) {
            pClr = &(pColorPaletteZ14[pWS[x]]);
            pD[0] = pClr->rgbBlue; //B
            pD[1] = pClr->rgbGreen; //G
            pD[2] = pClr->rgbRed; //R
            pD += 3;
        }
        pWSL += cx;
        // pDL -= nBPS;
        pDL += nBPS;
    }
}

#endif

}  // namespace

CameraPrivate::CameraPrivate(Camera *q)
    : q_ptr(q), etron_di_(nullptr), dev_sel_info_({-1}), stream_info_dev_index_(-1) {
    DBG_LOGD(__func__);

    int ret = EtronDI_Init(&etron_di_, false);
    DBG_LOGI("MYNT EYE Init: %d", ret);
    unused(ret);

    stream_color_info_ptr_ = (PETRONDI_STREAM_INFO)malloc(sizeof(ETRONDI_STREAM_INFO)*64);
    stream_depth_info_ptr_ = (PETRONDI_STREAM_INFO)malloc(sizeof(ETRONDI_STREAM_INFO)*64);
    color_res_index_ = 0;
    depth_res_index_ = 0;
#ifndef OS_WIN
    dtc_ = DEPTH_IMG_NON_TRANSFER;
#endif
    framerate_ = 30;

    color_serial_number_ = 0;
    depth_serial_number_ = 0;
    color_image_size_ = 0;
    depth_image_size_ = 0;
    color_img_buf_ = nullptr;
    color_rgb_buf_ = nullptr;
    depth_img_buf_ = nullptr;
    depth_rgb_buf_ = nullptr;

#ifdef OS_WIN
    DmColorMode14(color_palette_z14_, 0/*normal*/);
#endif
}

CameraPrivate::~CameraPrivate() {
    DBG_LOGD(__func__);
    EtronDI_Release(&etron_di_);

    free(stream_color_info_ptr_);
    free(stream_depth_info_ptr_);

    Close();
}

void CameraPrivate::GetDevices(std::vector<DeviceInfo> &dev_infos) {
    dev_infos.clear();

    int count = EtronDI_GetDeviceNumber(etron_di_);
    DBG_LOGD("EtronDI_GetDeviceNumber: %d", count);

    DEVSELINFO dev_sel_info;
    DEVINFORMATION *p_dev_info = (DEVINFORMATION*)malloc(sizeof(DEVINFORMATION)*count);

    for (int i = 0; i < count; i++) {
        dev_sel_info.index = i;

        EtronDI_GetDeviceInfo(etron_di_, &dev_sel_info, p_dev_info+i);

        char sz_buf[256];
        int actual_length = 0;
        if (ETronDI_OK == EtronDI_GetFwVersion(etron_di_, &dev_sel_info, sz_buf, 256, &actual_length)) {
            DeviceInfo info;
            info.index = i;
            info.name = p_dev_info[i].strDevName;
            info.type = p_dev_info[i].nDevType;
            info.pid = p_dev_info[i].wPID;
            info.vid = p_dev_info[i].wVID;
            info.chip_id = p_dev_info[i].nChipID;
            info.fw_version = sz_buf;
            dev_infos.push_back(std::move(info));
        }
    }

    free(p_dev_info);
}

void CameraPrivate::GetResolutions(const std::int32_t &dev_index,
        std::vector<StreamInfo> &color_infos, std::vector<StreamInfo> &depth_infos) {
    color_infos.clear();
    depth_infos.clear();

    memset(stream_color_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO)*64);
    memset(stream_depth_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO)*64);

    DEVSELINFO dev_sel_info{dev_index};
    EtronDI_GetDeviceResolutionList(etron_di_, &dev_sel_info, 64, stream_color_info_ptr_, 64, stream_depth_info_ptr_);

    PETRONDI_STREAM_INFO stream_temp_info_ptr = stream_color_info_ptr_;
    int i = 0;
    while (i < 64) {
        if (stream_temp_info_ptr->nWidth > 0) {
            StreamInfo info;
            info.index = i;
            info.width = stream_temp_info_ptr->nWidth;
            info.height = stream_temp_info_ptr->nHeight;
            info.format = stream_temp_info_ptr->bFormatMJPG ? StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV;
            color_infos.push_back(info);
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
            info.format = stream_temp_info_ptr->bFormatMJPG ? StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV;
            depth_infos.push_back(info);
        }
        stream_temp_info_ptr++;
        i++;
    }

    stream_info_dev_index_ = dev_index;
}

void CameraPrivate::GetResolutionIndex(const std::int32_t &dev_index, const StreamMode &stream_mode, 
    const StreamFormat &stream_format, int &color_res_index, int &depth_res_index) {

    color_res_index = -1;
    depth_res_index = -1;
    int width = 0, height = 0;
    switch (stream_mode) {
        case StreamMode::STREAM_1280x480:
            width = 1280;
            height = 480;
            break;
        case StreamMode::STREAM_1280x720:
            width = 1280;
            height = 720;
            break;
        case StreamMode::STREAM_2560x720:
            width = 2560;
            height = 720;
            break;
        // case StreamMode::STREAM_2560x960:
        //     width = 2560;
        //     height = 960;
        //     break;
        case StreamMode::STREAM_640x480:
            width = 640;
            height = 480;
            break;
        default:
            break;
    }

    std::string format;
    switch (stream_format) {
        case StreamFormat::STREAM_MJPG:
            format = "MJPG";
            break;
        case StreamFormat::STREAM_YUYV:
            format = "YUYV";
            break;
        default:
            break;
    }


    memset(stream_color_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO)*64);
    memset(stream_depth_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO)*64);

    DEVSELINFO dev_sel_info{dev_index};
    EtronDI_GetDeviceResolutionList(etron_di_, &dev_sel_info, 64, stream_color_info_ptr_, 64, stream_depth_info_ptr_);

    PETRONDI_STREAM_INFO stream_temp_info_ptr = stream_color_info_ptr_;
    int i = 0;
    while (i < 64) {
        if (stream_temp_info_ptr->nWidth == width && stream_temp_info_ptr->nHeight == height 
            && stream_format == (stream_temp_info_ptr->bFormatMJPG ? StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV)) {
            color_res_index = i;
            break;
        }
        stream_temp_info_ptr++;
        i++;
    }

    if (color_res_index == -1) {
        LOGE("Error: Color Mode width[%d] height[%d] format[%s] not support. Please check the resolution list.", width, height, format);
        color_res_index = 0;
    }

    stream_temp_info_ptr = stream_depth_info_ptr_;
    i = 0;
    while (i < 64) {
        if (stream_temp_info_ptr->nHeight == height 
            && stream_format == (stream_temp_info_ptr->bFormatMJPG ? StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV)) {
            depth_res_index = i;
            break;
        }
        stream_temp_info_ptr++;
        i++;
    }

    if (depth_res_index == -1) {
        LOGE("Error: Depth Mode width[%d] height[%d] format[%s] not support. Please check the resolution list.", width, height, format);
        depth_res_index = 0;
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

ErrorCode CameraPrivate::Open(const InitParams &params) {
    dev_sel_info_.index = params.dev_index;

    //if (params.dev_info.type == PUMA) {
        depth_data_type_ = 2;  // 1: 11 bits. 2: 14 bits
        EtronDI_SetDepthDataType(etron_di_, &dev_sel_info_, depth_data_type_);
        DBG_LOGI("SetDepthDataType: %d", depth_data_type_);
    //}

    SetAutoExposureEnabled(params.state_ae);
    SetAutoWhiteBalanceEnabled(params.state_awb);

    if (params.framerate > 0) framerate_ = params.framerate;
    LOGI("-- Framerate: %d", framerate_);
#ifndef OS_WIN
    std::string dtc_name = "Unknown";
    switch (params.depth_mode) {
        case DepthMode::DEPTH_NON:
            dtc_ = DEPTH_IMG_NON_TRANSFER;
            dtc_name = "Non";
            break;
        case DepthMode::DEPTH_GRAY:
            dtc_ = DEPTH_IMG_GRAY_TRANSFER;
            dtc_name = "Gray";
            break;
        case DepthMode::DEPTH_COLORFUL:
            dtc_ = DEPTH_IMG_COLORFUL_TRANSFER;
            dtc_name = "Colorful";
            break;
        case DepthMode::DEPTH_NON_16UC1:
            dtc_ = DEPTH_IMG_NON_TRANSFER;
            dtc_name = "Non 16UC1";
            break;
        case DepthMode::DEPTH_NON_8UC1:
            dtc_ = DEPTH_IMG_NON_TRANSFER;
            dtc_name = "Non 8UC1";
            break;
        default:
            break;
    }
#endif
    depth_mode_ = params.depth_mode;

    if (params.dev_index != stream_info_dev_index_) {
        std::vector<StreamInfo> color_infos;
        std::vector<StreamInfo> depth_infos;
        GetResolutions(params.dev_index, color_infos, depth_infos);
    }

    GetResolutionIndex(params.dev_index, params.stream_mode, params.stream_format, 
        color_res_index_, depth_res_index_);
    LOGI("-- Color Stream: %dx%d %s",
        stream_color_info_ptr_[color_res_index_].nWidth,
        stream_color_info_ptr_[color_res_index_].nHeight,
        stream_color_info_ptr_[color_res_index_].bFormatMJPG ? "MJPG" : "YUYV");
    LOGI("-- Depth Stream: %dx%d %s",
        stream_depth_info_ptr_[depth_res_index_].nWidth,
        stream_depth_info_ptr_[depth_res_index_].nHeight,
        stream_depth_info_ptr_[depth_res_index_].bFormatMJPG ? "MJPG" : "YUYV");

    if (depth_data_type_ != 1 && depth_data_type_ != 2) {
        throw std::runtime_error(format_string("Error: Depth data type (%d) not supported.", depth_data_type_));
    }

    if (params.ir_intensity >= 0) {
        if (SetFWRegister(0xE0, params.ir_intensity)) {
            LOGI("-- IR intensity: %d", params.ir_intensity);
        } else {
            LOGI("-- IR intensity: %d (failed)", params.ir_intensity);
        }
    }

    ReleaseBuf();

#ifdef OS_WIN
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
        return ErrorCode::SUCCESS;
    } else {
        dev_sel_info_.index = -1;  // reset flag
        return ErrorCode::ERROR_CAMERA_OPEN_FAILED;
    }
}

bool CameraPrivate::IsOpened() {
    return dev_sel_info_.index != -1;
}

#ifdef OS_WIN

void CameraPrivate::ImgCallback(EtronDIImageType::Value imgType, int imgId,
        unsigned char *imgBuf, int imgSize, int width, int height,
        int serialNumber, void *pParam) {
    CameraPrivate *p = static_cast<CameraPrivate *>(pParam);
    std::lock_guard<std::mutex> _(p->mtx_imgs_);

    if (EtronDIImageType::IsImageColor(imgType)) {
        // LOGI("Image callback color");
        if (!p->color_img_buf_) {
            unsigned int color_img_width  = (unsigned int)(p->stream_color_info_ptr_[p->color_res_index_].nWidth);
            unsigned int color_img_height = (unsigned int)(p->stream_color_info_ptr_[p->color_res_index_].nHeight);
            p->color_img_buf_ = (unsigned char*)calloc(color_img_width*color_img_height*3, sizeof(unsigned char));
        }

        memcpy(p->color_img_buf_, imgBuf, imgSize);

        p->is_color_rgb24_ = (imgType == EtronDIImageType::COLOR_RGB24);
        p->is_color_mjpg_ = (imgType == EtronDIImageType::COLOR_MJPG);
    } else if (EtronDIImageType::IsImageDepth(imgType)) {
        // LOGI("Image callback depth");
        if (!p->depth_img_buf_) {
            unsigned int depth_img_width  = (unsigned int)(p->stream_depth_info_ptr_[p->depth_res_index_].nWidth);
            unsigned int depth_img_height = (unsigned int)(p->stream_depth_info_ptr_[p->depth_res_index_].nHeight);
            p->depth_img_buf_ = (unsigned char*)calloc(depth_img_width*depth_img_height*2, sizeof(unsigned char));
            p->depth_data_size_ = depth_img_width*depth_img_height*2;
        }
        memcpy(p->depth_img_buf_, imgBuf, p->depth_data_size_);
    } else {
        LOGE("Image callback failed. Unknown image type.");
    }
}

ErrorCode CameraPrivate::RetrieveImage(cv::Mat &color, cv::Mat &depth) {
    if (!IsOpened())return ErrorCode::ERROR_CAMERA_NOT_OPENED;

    if (!color_img_buf_ || !depth_img_buf_) {
        return ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
    }

    std::lock_guard<std::mutex> _(mtx_imgs_);

    bool color_ok = false;
    if (color_img_buf_) {
        // LOGI("Retrieve image color");
        unsigned int color_img_width  = (unsigned int)(stream_color_info_ptr_[color_res_index_].nWidth);
        unsigned int color_img_height = (unsigned int)(stream_color_info_ptr_[color_res_index_].nHeight);
        if (is_color_mjpg_) {  // mjpg
            if (!color_rgb_buf_) {
                color_rgb_buf_ = (unsigned char*)calloc(color_img_width*color_img_height*3, sizeof(unsigned char));
            }
            MJPEG_TO_RGB24_LIBJPEG(color_img_buf_, color_image_size_, color_rgb_buf_);
            cv::Mat color_img(color_img_height, color_img_width, CV_8UC3, color_rgb_buf_);
            cv::cvtColor(color_img, color, CV_RGB2BGR);
            color_ok = true;
        } else if (is_color_rgb24_) {  // rgb24
            cv::Mat color_img(color_img_height, color_img_width, CV_8UC3, color_img_buf_);
            cv::flip(color_img, color, 0);
            color_ok = true;
        } else {
            LOGE("Unknown image color type.");
        }
    }

    bool depth_ok = false;
    if (depth_img_buf_) {  // DEPTH_14BITS for ETronDI_DEPTH_DATA_14_BITS
        // LOGI("Retrieve image depth");

        unsigned int depth_img_width  = (unsigned int)(stream_depth_info_ptr_[depth_res_index_].nWidth);
        unsigned int depth_img_height = (unsigned int)(stream_depth_info_ptr_[depth_res_index_].nHeight);

        depth_raw_ = cv::Mat(depth_img_height, depth_img_width, CV_16UC1, depth_img_buf_);

        switch (depth_mode_) {
            case DepthMode::DEPTH_GRAY:
            case DepthMode::DEPTH_NON_8UC1:
                cv::normalize(depth_raw_, depth, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                break;
            case DepthMode::DEPTH_COLORFUL:
                if (!depth_rgb_buf_) {
                    depth_rgb_buf_ = (unsigned char*)calloc(depth_img_width*depth_img_height*3, sizeof(unsigned char));
                }
                UpdateZ14DisplayImage_DIB24(color_palette_z14_, depth_img_buf_, depth_rgb_buf_, depth_img_width, depth_img_height);
                depth = cv::Mat(depth_img_height, depth_img_width, CV_8UC3, depth_rgb_buf_);
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
        const cv::Vec2b &pixel = depth_img.at<cv::Vec2b>(0,0);
        ushort depth_pixel = (pixel[0] & 0xff) | ((pixel[1] & 0xff) << 8);
        depth_min = depth_max = depth_pixel;
        // compute depth pixels
        for (int i = 0; i < h; ++i) {  // row
            for (int j = 0; j < w; ++j) {  // col
                const cv::Vec2b &pixel = depth_img.at<cv::Vec2b>(i,j);
                depth_pixel = (pixel[0] & 0xff) | ((pixel[1] & 0xff) << 8);
                depth_raw_.at<ushort>(i,j) = depth_pixel;
                if (depth_pixel < depth_min) depth_min = depth_pixel;
                if (depth_pixel > depth_max) depth_max = depth_pixel;
            }
        }
        */
    }

    if (color_ok && depth_ok) {
        return ErrorCode::SUCCESS;
    } else {
        return ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
    }
}

#else

ErrorCode CameraPrivate::RetrieveImage(cv::Mat &color, cv::Mat &depth) {
//ErrorCode CameraPrivate::RetrieveImage(cv::Mat &mat, const View &view) {
    if (!IsOpened())return ErrorCode::ERROR_CAMERA_NOT_OPENED;

    unsigned int color_img_width  = (unsigned int)(stream_color_info_ptr_[color_res_index_].nWidth);
    unsigned int color_img_height = (unsigned int)(stream_color_info_ptr_[color_res_index_].nHeight);
    unsigned int depth_img_width  = (unsigned int)(stream_depth_info_ptr_[depth_res_index_].nWidth);
    unsigned int depth_img_height = (unsigned int)(stream_depth_info_ptr_[depth_res_index_].nHeight);
    bool is_mjpeg = stream_color_info_ptr_[color_res_index_].bFormatMJPG;

    if (!color_img_buf_) {
        color_img_buf_ = (unsigned char*)calloc(color_img_width*color_img_height*2, sizeof(unsigned char));
    }
    if (is_mjpeg && !color_rgb_buf_) {
        color_rgb_buf_ = (unsigned char*)calloc(color_img_width*color_img_height*3, sizeof(unsigned char));
    }
    if (!depth_img_buf_) {
        if (dtc_ == DEPTH_IMG_COLORFUL_TRANSFER || dtc_ == DEPTH_IMG_GRAY_TRANSFER) {
            depth_img_buf_ = (unsigned char*)calloc(depth_img_width*2*depth_img_height*3, sizeof(unsigned char));
        } else {
            depth_img_buf_ = (unsigned char*)calloc(depth_img_width*depth_img_height*2, sizeof(unsigned char));
        }
    }

    int ret = EtronDI_Get2Image(etron_di_, &dev_sel_info_,
        (BYTE*)color_img_buf_, (BYTE*)depth_img_buf_,
        &color_image_size_, &depth_image_size_,
        &color_serial_number_, &depth_serial_number_, depth_data_type_);

    if (ETronDI_OK != ret) {
        DBG_LOGI("EtronDI_Get2Image: %d", ret);
        return ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
    }

    // color
    if (is_mjpeg) {
        MJPEG_TO_RGB24_LIBJPEG(color_img_buf_, color_image_size_, color_rgb_buf_);
        cv::Mat color_img(color_img_height, color_img_width, CV_8UC3, color_rgb_buf_);
        cv::cvtColor(color_img, color, CV_RGB2BGR);
    } else {
        cv::Mat color_img(color_img_height, color_img_width, CV_8UC2, color_img_buf_);
        cv::cvtColor(color_img, color, CV_YUV2BGR_YUY2);
    }

    // depth
    if (dtc_ == DEPTH_IMG_COLORFUL_TRANSFER || dtc_ == DEPTH_IMG_GRAY_TRANSFER) {
        // Depth data type: 11 bits & 14 bits
        cv::Mat depth_img(depth_img_height, depth_img_width, CV_8UC3, depth_img_buf_);
        cv::cvtColor(depth_img, depth, CV_RGB2BGR);
    } else {  // DEPTH_IMG_NON_TRANSFER
        cv::Mat depth_img(depth_img_height, depth_img_width, CV_8UC2, depth_img_buf_);
        if (depth_mode_ == DepthMode::DEPTH_NON) {
            cv::cvtColor(depth_img, depth, CV_YUV2BGR_YUY2);
        } else {
            const int h = static_cast<int>(depth_img_height);
            const int w = static_cast<int>(depth_img_width);
            if (depth_raw_.rows != h || depth_raw_.cols != w) {
                depth_raw_ = cv::Mat(h, w, CV_16UC1);
            }
            // initialize depth min,max
            const cv::Vec2b &pixel = depth_img.at<cv::Vec2b>(0,0);
            ushort depth_pixel = (pixel[0] & 0xff) | ((pixel[1] & 0xff) << 8);
            depth_min = depth_max = depth_pixel;
            // compute depth pixels
            for (int i = 0; i < h; ++i) {  // row
                for (int j = 0; j < w; ++j) {  // col
                    const cv::Vec2b &pixel = depth_img.at<cv::Vec2b>(i,j);
                    depth_pixel = (pixel[0] & 0xff) | ((pixel[1] & 0xff) << 8);
                    depth_raw_.at<ushort>(i,j) = depth_pixel;
                    if (depth_pixel < depth_min) depth_min = depth_pixel;
                    if (depth_pixel > depth_max) depth_max = depth_pixel;
                }
            }

            if (depth_mode_ == DepthMode::DEPTH_NON_16UC1) {
                depth = depth_raw_;
            } else if (depth_mode_ == DepthMode::DEPTH_NON_8UC1) {
                if (depth.rows != h || depth.cols != w || depth.type() != CV_8UC1) {
                    depth = cv::Mat(h, w, CV_8UC1);
                }
                // transfer depth to gray
                ushort depth_dist = depth_max - depth_min;
                for (int i = 0; i < h; ++i) {  // row
                    for (int j = 0; j < w; ++j) {  // col
                        const ushort depth_pixel = depth_raw_.at<ushort>(i,j);
                        depth.at<uchar>(i,j) = 255 * (depth_pixel - depth_min) / depth_dist;
                    }
                }
            } else {
                throw new std::runtime_error("Error: Depth mode is not supported.");
            }
        }
    }
    return ErrorCode::SUCCESS;
    /*
    switch (view) {
        case View::VIEW_IMAGE:
            return RetrieveColorImage(mat);
        case View::VIEW_DEPTH:
            return RetrieveDepthImage(mat);
        default:
            LOGE("Error: Retrieve view type not supported.");
            return ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
    }
    */
}

#endif

/*
ErrorCode CameraPrivate::RetrieveColorImage(cv::Mat &mat) {
    int ret = EtronDI_GetImage(etron_di_, &dev_sel_info_,
        (BYTE*)color_img_buf_, &color_image_size_, &color_serial_number_, 0);
    if (ETronDI_OK != ret) {
        DBG_LOGI("EtronDI_GetImage: %d", ret);
        return ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
    }

    unsigned int color_img_width  = (unsigned int)(stream_color_info_ptr_[color_res_index_].nWidth);
    unsigned int color_img_height = (unsigned int)(stream_color_info_ptr_[color_res_index_].nHeight);

    bool is_mjpeg = stream_color_info_ptr_[color_res_index_].bFormatMJPG;
    if (is_mjpeg) {
        MJPEG_TO_RGB24_LIBJPEG(color_img_buf_, color_image_size_, color_rgb_buf_);
        cv::Mat img(color_img_height, color_img_width, CV_8UC3, color_rgb_buf_);
        cv::cvtColor(img, mat, CV_RGB2BGR);
    } else {
        cv::Mat img(color_img_height, color_img_width, CV_8UC2, color_img_buf_);
        cv::cvtColor(img, mat, CV_YUV2BGR_YUY2);
    }
    return ErrorCode::SUCCESS;
}
*/

/*
ErrorCode CameraPrivate::RetrieveDepthImage(cv::Mat &mat) {
    int ret = EtronDI_GetImage(etron_di_, &dev_sel_info_,
        (BYTE*)depth_img_buf_, &depth_image_size_, &depth_serial_number_, depth_data_type_);
    if (ETronDI_OK != ret) {
        DBG_LOGI("EtronDI_GetImage: %d", ret);
        return ErrorCode::ERROR_CAMERA_RETRIEVE_FAILED;
    }

    unsigned int depth_img_width  = (unsigned int)(stream_depth_info_ptr_[depth_res_index_].nWidth);
    unsigned int depth_img_height = (unsigned int)(stream_depth_info_ptr_[depth_res_index_].nHeight);

    if (dtc_ == DEPTH_IMG_COLORFUL_TRANSFER || dtc_ == DEPTH_IMG_GRAY_TRANSFER) {
        // Depth data type: 11 bits & 14 bits
        cv::Mat img(depth_img_height, depth_img_width, CV_8UC3, depth_img_buf_);
        cv::cvtColor(img, mat, CV_RGB2BGR);
    } else {  // DEPTH_IMG_NON_TRANSFER
        cv::Mat img(depth_img_height, depth_img_width, CV_8UC2, depth_img_buf_);
        cv::cvtColor(img, mat, CV_YUV2BGR_YUY2);
    }

    return ErrorCode::SUCCESS;
}
*/

void CameraPrivate::Close() {
    if (dev_sel_info_.index != -1) {
        EtronDI_CloseDevice(etron_di_, &dev_sel_info_);
        dev_sel_info_.index = -1;
    }
    ReleaseBuf();
}

void CameraPrivate::ReleaseBuf() {
    if (color_img_buf_) {
        delete color_img_buf_;
        color_img_buf_ = nullptr;
    }
    if (color_rgb_buf_) {
        delete color_rgb_buf_;
        color_rgb_buf_ = nullptr;
    }
    if (depth_img_buf_) {
        delete depth_img_buf_;
        depth_img_buf_ = nullptr;
    }
    if (depth_rgb_buf_) {
        delete depth_rgb_buf_;
        depth_rgb_buf_ = nullptr;
    }
}

bool CameraPrivate::GetSensorRegister(int id, unsigned short address, unsigned short *value, int flag) {
    if (!IsOpened()) throw std::runtime_error("Error: Camera not opened.");
    #ifdef OS_WIN
    return ETronDI_OK == EtronDI_GetSensorRegister(etron_di_, &dev_sel_info_, id, address, value, flag, 2);
    #else
    return ETronDI_OK == EtronDI_GetSensorRegister(etron_di_, &dev_sel_info_, id, address, value, flag, SENSOR_BOTH);
    #endif
}

bool CameraPrivate::GetHWRegister(unsigned short address, unsigned short *value, int flag) {
    if (!IsOpened()) throw std::runtime_error("Error: Camera not opened.");
    return ETronDI_OK == EtronDI_GetHWRegister(etron_di_, &dev_sel_info_, address, value, flag);
}

bool CameraPrivate::GetFWRegister(unsigned short address, unsigned short *value, int flag) {
    if (!IsOpened()) throw std::runtime_error("Error: Camera not opened.");
    return ETronDI_OK == EtronDI_GetFWRegister(etron_di_, &dev_sel_info_, address, value, flag);
}

bool CameraPrivate::SetSensorRegister(int id, unsigned short address, unsigned short value, int flag) {
    if (!IsOpened()) throw std::runtime_error("Error: Camera not opened.");
    #ifdef OS_WIN
    return ETronDI_OK == EtronDI_SetSensorRegister(etron_di_, &dev_sel_info_, id, address, value, flag, 2);
    #else
    return ETronDI_OK == EtronDI_SetSensorRegister(etron_di_, &dev_sel_info_, id, address, value, flag, SENSOR_BOTH);
    #endif
}

bool CameraPrivate::SetHWRegister(unsigned short address, unsigned short value, int flag) {
    if (!IsOpened()) throw std::runtime_error("Error: Camera not opened.");
    return ETronDI_OK == EtronDI_SetHWRegister(etron_di_, &dev_sel_info_, address, value, flag);
}

bool CameraPrivate::SetFWRegister(unsigned short address, unsigned short value, int flag) {
    if (!IsOpened()) throw std::runtime_error("Error: Camera not opened.");
    return ETronDI_OK == EtronDI_SetFWRegister(etron_di_, &dev_sel_info_, address, value, flag);
}

int CameraPrivate::MJPEG_TO_RGB24_LIBJPEG(unsigned char *jpg, int nJpgSize, unsigned char *rgb) {
    struct jpeg_decompress_struct cinfo;
    struct my_error_mgr jerr;

    int rc;
    int row_stride, width, height, pixel_size;

    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_exit;

    if (setjmp(jerr.setjmp_buffer)) {
        jpeg_destroy_decompress(&cinfo);
        return 0;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, jpg, nJpgSize);

    rc = jpeg_read_header(&cinfo, TRUE);

    if (rc != 1) {
        LOGE("Error: File does not seem to be a normal JPEG !!");
    }

    jpeg_start_decompress(&cinfo);

    width = cinfo.output_width;
    height = cinfo.output_height;
    pixel_size = cinfo.output_components;

    row_stride = width * pixel_size;

    while (cinfo.output_scanline < cinfo.output_height) {
        unsigned char *buffer_array[1];
        //buffer_array[0] = rgb + (width * height * 3) - (cinfo.output_scanline) * row_stride;
        buffer_array[0] = rgb + (cinfo.output_scanline) * row_stride;

        jpeg_read_scanlines(&cinfo, buffer_array, 1);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    unused(height);
    return 0;
}
