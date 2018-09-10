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
#include "mynteye/image.h"

#include "mynteye/util/convertor.h"
#include "mynteye/util/log.h"

MYNTEYE_USE_NAMESPACE

namespace {

// bytes_per_pixel
int get_image_bpp(const ImageFormat& format) {
  switch (format) {
    case ImageFormat::IMAGE_BGR_24: return 3;
    case ImageFormat::IMAGE_RGB_24: return 3;
    case ImageFormat::IMAGE_GRAY_8: return 1;
    case ImageFormat::IMAGE_GRAY_16: return 2;
    case ImageFormat::IMAGE_GRAY_24: return 3;
    case ImageFormat::IMAGE_YUYV: return 2;
    case ImageFormat::IMAGE_MJPG: return 2;
    default: throw new std::runtime_error("ImageFormat not supported");
  }
}

int get_image_size(const ImageFormat& format, const int& width,
    const int& height) {
  return width * height * get_image_bpp(format);
}

#ifdef WITH_OPENCV
int get_mat_type(const ImageFormat& format) {
  switch (format) {
    case ImageFormat::IMAGE_BGR_24: return CV_8UC3;
    case ImageFormat::IMAGE_RGB_24: return CV_8UC3;
    case ImageFormat::IMAGE_GRAY_8: return CV_8UC1;
    case ImageFormat::IMAGE_GRAY_16: return CV_16UC1;
    case ImageFormat::IMAGE_GRAY_24: return CV_8UC3;
    case ImageFormat::IMAGE_YUYV: return CV_8UC2;
    default: throw new std::runtime_error("ImageFormat not support to cv::Mat");
  }
}
#endif

}  // namespace

Image::Image(ImageType type, ImageFormat format, int width, int height,
    bool is_buffer)
  : type_(type),
    format_(format),
    width_(width),
    height_(height),
    is_buffer_(is_buffer),
    raw_format_(format) {
  auto n = get_image_size(format, width, height);
  data_.assign(n, 0);
  set_valid_size(n);
}

Image::~Image() {
}

Image::pointer Image::Create(ImageType type, ImageFormat format, int width,
    int height, bool is_buffer) {
  switch (type) {
    case ImageType::IMAGE_COLOR:
      return ImageColor::Create(format, width, height, is_buffer);
    case ImageType::IMAGE_DEPTH:
      return ImageDepth::Create(format, width, height, is_buffer);
    default:
      throw new std::runtime_error("ImageType must be color or depth");
  }
}

#ifdef WITH_OPENCV
cv::Mat Image::ToMat() {
  return cv::Mat(height_, width_, get_mat_type(format_), data());
}
#endif

Image::pointer Image::Clone() const {
  auto image = Create(type_, format_, width_, height_, false);
  std::copy(data_.begin(), data_.end(), image->data_.begin());
  image->set_valid_size(valid_size_);
  return image;
}

Image::pointer Image::GetCache(const ImageFormat& format) {
  auto bpp = get_image_bpp(format);
  if (bpp_caches_.find(bpp) != bpp_caches_.end()) {
    return bpp_caches_[bpp];
  }
  auto image = Create(type_, format, width_, height_, false);
  bpp_caches_[bpp] = image;
  return image;
}

bool Image::ResetBuffer() {
  if (is_buffer_) {
    format_ = raw_format_;
    return true;
  }
  LOGW("Reset buffer, but it's not a buffer.");
  return false;
}

// ImageColor

ImageColor::ImageColor(ImageFormat format, int width, int height,
    bool is_buffer)
  : Image(ImageType::IMAGE_COLOR, format, width, height, is_buffer) {
}

ImageColor::~ImageColor() {
}

Image::pointer ImageColor::To(ImageFormat format) {
  // LOGI(strings::format_string("color src: %d, dst: %d", format_, format));
  if (format == format_) {
    return shared_from_this();
  }
  switch (format_) {  // src
    case ImageFormat::COLOR_BGR:
      if (format == ImageFormat::COLOR_RGB) {
        BGR_TO_RGB(data(), width_, height_);
        format_ = format;
        return shared_from_this();
      }
      break;
    case ImageFormat::COLOR_RGB:
      if (format == ImageFormat::COLOR_BGR) {
        RGB_TO_BGR(data(), width_, height_);
        format_ = format;
        return shared_from_this();
      }
      break;
    case ImageFormat::COLOR_YUYV:
      if (format == ImageFormat::COLOR_RGB) {
        auto image = GetCache(format);
        YUYV_TO_RGB(data(), image->data(), width_, height_);
        return image;
      } else if (format == ImageFormat::COLOR_BGR) {
        auto image = GetCache(format);
        YUYV_TO_BGR(data(), image->data(), width_, height_);
        return image;
      }
      break;
    case ImageFormat::COLOR_MJPG:
      if (format == ImageFormat::COLOR_RGB) {
        auto image = GetCache(format);
        MJPEG_TO_RGB_LIBJPEG(data(), valid_size_, image->data());
        return image;
      } else if (format == ImageFormat::COLOR_BGR) {
        return To(ImageFormat::COLOR_RGB)->To(ImageFormat::COLOR_BGR);
      }
      break;
    default: break;
  }
  throw new std::runtime_error(strings::format_string(
      "Can not convert from %s to %s", format_, format));
}

// ImageDepth

ImageDepth::ImageDepth(ImageFormat format, int width, int height,
    bool is_buffer)
  : Image(ImageType::IMAGE_DEPTH, format, width, height, is_buffer) {
}

ImageDepth::~ImageDepth() {
}

Image::pointer ImageDepth::To(ImageFormat format) {
  // LOGI(strings::format_string("depth src: %d, dst: %d", format_, format));
  if (format == format_) {
    return shared_from_this();
  }
  switch (format_) {  // src
    case ImageFormat::DEPTH_RAW:
      if (format == ImageFormat::DEPTH_GRAY) {
        std::uint16_t* depths = reinterpret_cast<std::uint16_t*>(data());
        std::uint16_t depth, depth_min, depth_max;
        depth = depth_min = depth_max = *(depths);
        for (int i = 0; i < height_; ++i) {  // row
          for (int j = 0; j < width_; ++j) {  // col
            depth = *(depths + (i * width_) + j);
            if (depth < depth_min) depth_min = depth;
            if (depth > depth_max) depth_max = depth;
          }
        }

        auto image = GetCache(format);
        auto data = image->data();
        int offset;
        std::uint16_t depth_dist = depth_max - depth_min;
        for (int i = 0; i < height_; ++i) {  // row
          for (int j = 0; j < width_; ++j) {  // col
            offset = (i * width_) + j;
            depth = *(depths + offset);
            *(data + offset) = 255 * (depth - depth_min) / depth_dist;
          }
        }
        return image;
      }
      break;
    case ImageFormat::DEPTH_BGR:
      if (format == ImageFormat::DEPTH_RGB) {
        BGR_TO_RGB(data(), width_, height_);
        format_ = format;
        return shared_from_this();
      }
      break;
    case ImageFormat::DEPTH_RGB:
      if (format == ImageFormat::DEPTH_BGR) {
        RGB_TO_BGR(data(), width_, height_);
        format_ = format;
        return shared_from_this();
      }
      break;
    default: break;
  }
  throw new std::runtime_error(strings::format_string(
      "Can not convert from %s to %s", format_, format));
}
