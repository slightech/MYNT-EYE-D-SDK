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
#ifndef MYNTEYE_UTIL_CONVERTOR_H_
#define MYNTEYE_UTIL_CONVERTOR_H_
#pragma once

#include <stdio.h>
#include <setjmp.h>

extern "C" {

#include <jpeglib.h>

}

#include "mynteye/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

struct my_error_mgr {
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

typedef struct my_error_mgr* my_error_ptr;

extern int MJPEG_TO_RGB24_LIBJPEG(unsigned char* jpg, int nJpgSize,
    unsigned char* rgb);

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_UTIL_CONVERTOR_H_
