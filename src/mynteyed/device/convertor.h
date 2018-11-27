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
#ifndef MYNTEYE_DEVICE_CONVERTOR_H_
#define MYNTEYE_DEVICE_CONVERTOR_H_
#pragma once

#ifdef WITH_JPEG

#include <stdio.h>
#include <setjmp.h>

extern "C" {

#include <jpeglib.h>

}

#endif

#include "mynteyed/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

#ifdef WITH_JPEG

struct my_error_mgr {
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

typedef struct my_error_mgr* my_error_ptr;

#endif

extern int MJPEG_TO_RGB_LIBJPEG(unsigned char* jpg, int nJpgSize,
    unsigned char* rgb);

extern int RGB_TO_RGB_LEFT(unsigned char* orig, unsigned char* left,
    unsigned int width, unsigned int height);
extern int RGB_TO_RGB_RIGHT(unsigned char* orig, unsigned char* right,
    unsigned int width, unsigned int height);

extern int RGB_TO_BGR_LEFT(unsigned char* orig, unsigned char* left,
    unsigned int width, unsigned int height);
extern int RGB_TO_BGR_RIGHT(unsigned char* orig, unsigned char* right,
    unsigned int width, unsigned int height);

extern int YUYV_TO_RGB(unsigned char* yuv, unsigned char* rgb,
    unsigned int width, unsigned int height);
extern int YUYV_TO_RGB_LEFT(unsigned char* yuv, unsigned char* rgb,
    unsigned int width, unsigned int height);
extern int YUYV_TO_RGB_RIGHT(unsigned char* yuv, unsigned char* rgb,
    unsigned int width, unsigned int height);

extern int YUYV_TO_BGR(unsigned char* yuv, unsigned char* bgr,
    unsigned int width, unsigned int height);
extern int YUYV_TO_BGR_LEFT(unsigned char* yuv, unsigned char* bgr,
    unsigned int width, unsigned int height);
extern int YUYV_TO_BGR_RIGHT(unsigned char* yuv, unsigned char* bgr,
    unsigned int width, unsigned int height);

extern void RGB_TO_BGR(unsigned char* rgb,
    unsigned int width, unsigned int height);

extern void BGR_TO_RGB(unsigned char* bgr,
    unsigned int width, unsigned int height);

extern void FLIP_UP_DOWN_C3(unsigned char* rgb, unsigned int width,
                            unsigned int height);

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_CONVERTOR_H_
