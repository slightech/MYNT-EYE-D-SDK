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
#include "util/cv_painter.h"

#include <opencv2/imgproc/imgproc.hpp>

#define FONT_FACE cv::FONT_HERSHEY_PLAIN
#define FONT_SCALE 1
#define FONT_COLOR cv::Scalar(255, 255, 255)
#define THICKNESS 1

namespace mynteye {
namespace util {

cv::Rect draw(
    const cv::Mat &img, const std::string &text,
    const gravity_t &gravity, const int &margin,
    const int &offset_x, const int &offset_y) {
  int w = img.cols, h = img.rows;

  int baseline = 0;
  cv::Size textSize =
      cv::getTextSize(text, FONT_FACE, FONT_SCALE, THICKNESS, &baseline);

  int x, y;
  switch (gravity) {
    case TOP_LEFT:
      x = margin;
      y = margin + textSize.height;
      break;
    case TOP_RIGHT:
      x = w - margin - textSize.width;
      y = margin + textSize.height;
      break;
    case BOTTOM_LEFT:
      x = margin;
      y = h - margin;
      break;
    case BOTTOM_RIGHT:
      x = w - margin - textSize.width;
      y = h - margin;
      break;
    default:  // TOP_LEFT
      x = margin;
      y = margin + textSize.height;
      break;
  }
  x += offset_x;
  y += offset_y;

  cv::Point org(x, y);
#ifdef USE_OPENCV2
  cv::putText(
      const_cast<cv::Mat &>(img), text, org, FONT_FACE, FONT_SCALE, FONT_COLOR,
      THICKNESS);
#else
  cv::putText(img, text, org, FONT_FACE, FONT_SCALE, FONT_COLOR, THICKNESS);
#endif
  return cv::Rect(org, textSize);
}

}  // namespace util
}  // namespace mynteye
