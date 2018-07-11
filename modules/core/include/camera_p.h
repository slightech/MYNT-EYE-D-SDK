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
#ifndef MYNTEYE_CORE_CAMERA_P_H_
#define MYNTEYE_CORE_CAMERA_P_H_
#pragma once

#include "camera.h"

#include "eSPDI.h"

#include <setjmp.h>

extern "C" {

#include <jpeglib.h>

}

namespace mynteye {

struct my_error_mgr {
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

typedef struct my_error_mgr *my_error_ptr;

class CameraPrivate {
public:
    CameraPrivate(Camera *q);
    /*
    CameraPrivate(const CameraPrivate &other);
    CameraPrivate(CameraPrivate &&other);
    CameraPrivate &operator=(const CameraPrivate &other);
    */
    ~CameraPrivate();

    void GetDevices(std::vector<DeviceInfo> &dev_infos);
    void GetResolutions(const std::int32_t &dev_index,
        std::vector<StreamInfo> &color_infos, std::vector<StreamInfo> &depth_infos);

    ErrorCode SetAutoExposureEnabled(bool enabled);
    ErrorCode SetAutoWhiteBalanceEnabled(bool enabled);

    bool GetSensorRegister(int id, unsigned short address, unsigned short *value, int flag = FG_Address_1Byte);
    bool GetHWRegister(unsigned short address, unsigned short *value, int flag = FG_Address_1Byte);
    bool GetFWRegister(unsigned short address, unsigned short *value, int flag = FG_Address_1Byte);

    bool SetSensorRegister(int id, unsigned short address, unsigned short value, int flag = FG_Address_1Byte);
    bool SetHWRegister(unsigned short address, unsigned short value, int flag = FG_Address_1Byte);
    bool SetFWRegister(unsigned short address, unsigned short value, int flag = FG_Address_1Byte);

    ErrorCode Open(const InitParams &params);

    bool IsOpened();

    ErrorCode RetrieveImage(cv::Mat &color, cv::Mat &depth);
    //ErrorCode RetrieveImage(cv::Mat &mat, const View &view);

    void Close();

    /** q-ptr that points to the API class */
    Camera *q_ptr;

private:
    //ErrorCode RetrieveColorImage(cv::Mat &mat);
    //ErrorCode RetrieveDepthImage(cv::Mat &mat);

    void ReleaseBuf();

    int MJPEG_TO_RGB24_LIBJPEG(unsigned char *jpg, int nJpgSize, unsigned char *rgb);

    void *etron_di_;

    DEVSELINFO dev_sel_info_;
    int depth_data_type_;

    PETRONDI_STREAM_INFO stream_color_info_ptr_;
    PETRONDI_STREAM_INFO stream_depth_info_ptr_;
    int color_res_index_;
    int depth_res_index_;
#ifndef OS_WIN
    DEPTH_TRANSFER_CTRL dtc_;
#endif
    int framerate_;

    std::int32_t stream_info_dev_index_;

    int color_serial_number_;
    int depth_serial_number_;
    unsigned long int color_image_size_;
    unsigned long int depth_image_size_;
    unsigned char *color_img_buf_;
    unsigned char *color_rgb_buf_;
    unsigned char *depth_img_buf_;

    DepthMode depth_mode_;
    cv::Mat depth_raw_;
    ushort depth_min;
    ushort depth_max;
};

}  // namespace mynteye

#endif  // MYNTEYE_CORE_CAMERA_P_H_
