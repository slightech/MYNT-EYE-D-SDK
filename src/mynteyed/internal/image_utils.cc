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
#include "mynteyed/internal/image_utils.h"

#include <algorithm>
#include <vector>

#include "mynteyed/util/log.h"

MYNTEYE_BEGIN_NAMESPACE

namespace images {

inline void _copy_left_yuyv(const std::uint8_t *in, std::uint8_t *out,
    int width, int height) {
  for (int i = 0; i < height; i++) {
    std::copy(in + i * width * 2,
        in + (width) + i * width * 2, out + i * width);
  }
}

inline void _copy_right_yuyv(const std::uint8_t *in, std::uint8_t *out,
    int width, int height) {
  for (int i = 0; i < height; i++) {
    std::copy(in + i * width * 2 + width,
        in + (i + 1) * width * 2, out + i * width);
  }
}

Image::pointer split_left_color(Image::pointer color) {
  if (color->format() != ImageFormat::COLOR_YUYV) {
    throw_error("Only support split color with yuyv format.");
  }
  auto image = ImageColor::Create(ImageType::IMAGE_LEFT_COLOR,
      color->format(), color->width() / 2, color->height(), false);
  image->set_frame_id(color->frame_id());
  _copy_left_yuyv(color->data(), image->data(),
      color->width(), color->height());
  return image;
}

Image::pointer split_right_color(Image::pointer color) {
  if (color->format() != ImageFormat::COLOR_YUYV) {
    throw_error("Only support split color with yuyv format.");
  }
  auto image = ImageColor::Create(ImageType::IMAGE_RIGHT_COLOR,
      color->format(), color->width() / 2, color->height(), false);
  image->set_frame_id(color->frame_id());
  _copy_right_yuyv(color->data(), image->data(),
      color->width(), color->height());
  return image;
}

}  // namespace images

MYNTEYE_END_NAMESPACE
