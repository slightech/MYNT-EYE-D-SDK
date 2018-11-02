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
#ifndef MYNTEYE_SAMPLES_UTIL_CV_PAINTER_H_
#define MYNTEYE_SAMPLES_UTIL_CV_PAINTER_H_
#pragma once

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/core/version.hpp>

#if CV_VERSION_EPOCH == 2
#define USE_OPENCV2
#endif

namespace mynteye {
namespace util {

typedef enum Gravity {
  TOP_LEFT,
  TOP_RIGHT,
  BOTTOM_LEFT,
  BOTTOM_RIGHT
} gravity_t;

cv::Rect draw(
    const cv::Mat &img, const std::string &text,
    const gravity_t &gravity = TOP_LEFT, const int &margin = 5,
    const int &offset_x = 0, const int &offset_y = 0);

}  // namespace util
}  // namespace mynteye

#endif  // MYNTEYE_SAMPLES_UTIL_CV_PAINTER_H_
