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

}  // namespace

CameraPrivate::CameraPrivate(Camera *q)
    : q_ptr(q), etron_di_(nullptr), dev_sel_info_({-1}), stream_info_dev_index_(-1) {
    DBG_LOGD(__func__);

    int ret = EtronDI_Init(&etron_di_, false);
    DBG_LOGI("EtronDI_Init: %d", ret);
    unused(ret);

    stream_color_info_ptr_ = (PETRONDI_STREAM_INFO)malloc(sizeof(ETRONDI_STREAM_INFO)*64);
    stream_depth_info_ptr_ = (PETRONDI_STREAM_INFO)malloc(sizeof(ETRONDI_STREAM_INFO)*64);
    color_res_index_ = 0;
    depth_res_index_ = 0;
    dtc_ = DEPTH_IMG_NON_TRANSFER;
    framerate_ = 30;

    color_serial_number_ = 0;
    depth_serial_number_ = 0;
    color_image_size_ = 0;
    depth_image_size_ = 0;
    color_img_buf_ = nullptr;
    color_rgb_buf_ = nullptr;
    depth_img_buf_ = nullptr;
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
        DBG_LOGI("EtronDI_SetDepthDataType: %d", depth_data_type_);
    //}

    SetAutoExposureEnabled(params.state_ae);
    SetAutoWhiteBalanceEnabled(params.state_awb);

    if (params.framerate > 0) framerate_ = params.framerate;
    LOGI("-- Framerate: %d", framerate_);

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
        default:
            break;
    }
    LOGI("-- Depth mode: %s", dtc_name);

    if (params.dev_index != stream_info_dev_index_) {
        std::vector<StreamInfo> color_infos;
        std::vector<StreamInfo> depth_infos;
        GetResolutions(params.dev_index, color_infos, depth_infos);
    }
    if (params.color_info_index > -1) {
        color_res_index_ = params.color_info_index;
    }
    if (params.depth_info_index > -1) {
        depth_res_index_ = params.depth_info_index;
    }
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

    if (params.ir_intensity > 0) {
        if (SetFWRegister(0xE0, params.ir_intensity)) {
            LOGI("-- IR intensity: %d", params.ir_intensity);
        } else {
            LOGI("-- IR intensity: %d (failed)", params.ir_intensity);
        }
    }

    ReleaseBuf();

    int ret = EtronDI_OpenDevice2(etron_di_, &dev_sel_info_,
        stream_color_info_ptr_[color_res_index_].nWidth,
        stream_color_info_ptr_[color_res_index_].nHeight,
        stream_color_info_ptr_[color_res_index_].bFormatMJPG,
        stream_depth_info_ptr_[depth_res_index_].nWidth,
        stream_depth_info_ptr_[depth_res_index_].nHeight,
        dtc_, false, NULL, &framerate_);

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
        //cv::cvtColor(depth_img, depth, CV_YUV2BGR_YUY2);
        const int h = static_cast<int>(depth_img_height);
        const int w = static_cast<int>(depth_img_width);
        const int size = h * w;
        if (depth.rows != h || depth.cols != w || depth.type() != CV_8UC1) {
            depth = cv::Mat(h, w, CV_8UC1);
        }
        if (depth_raw_.rows != h || depth_raw_.cols != w) {
            depth_raw_ = cv::Mat(h, w, CV_16UC1);
        }
        uchar *data = depth_img.data;
        // first depth pixel
        ushort depth_pixel = (data[0] & 0xff) | ((data[1] & 0xff) << 8);
        depth_raw_.data[0] = depth_min = depth_max = depth_pixel;
        // other depth pixels
        for (int i = 1; i < size; ++i) {
            depth_pixel = (data[i*2] & 0xff) | ((data[i*2 + 1] & 0xff) << 8);  // ushort
            depth_raw_.data[i] = depth_pixel;
            if (depth_pixel < depth_min) depth_min = depth_pixel;
            if (depth_pixel > depth_max) depth_max = depth_pixel;
        }
        ushort depth_dist = depth_max - depth_min;
        for (int i = 0; i < size; ++i) {
            depth.data[i] = static_cast<uchar>(255 * (depth_raw_.data[i] - depth_min) / depth_dist);
        }
        //cv::normalize(depth_raw_, depth, 0, 255, cv::NORM_MINMAX, CV_8UC1);
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
}

bool CameraPrivate::GetSensorRegister(int id, unsigned short address, unsigned short *value, int flag) {
    if (!IsOpened()) throw std::runtime_error("Error: Camera not opened.");
    return ETronDI_OK == EtronDI_GetSensorRegister(etron_di_, &dev_sel_info_, id, address, value, flag, SENSOR_BOTH);
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
    return ETronDI_OK == EtronDI_SetSensorRegister(etron_di_, &dev_sel_info_, id, address, value, flag, SENSOR_BOTH);
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
