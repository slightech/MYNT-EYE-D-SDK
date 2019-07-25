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
#include "mynteyed/device/image.h"

#include "mynteyed/device/convertor.h"
#include "mynteyed/device/data_caches.h"
// #include "mynteyed/internal/image_utils.h"
#include "mynteyed/util/log.h"

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
    // The valid size of MJPG will much smaller.
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

// Global data caches
DataCaches g_data_caches;
DataCaches g_left_data_caches;
DataCaches g_right_data_caches;
DataCaches g_depth_data_caches;

void init_cache_proper_sizes() {
  std::set<std::size_t> sizes;

  // all stream size
  std::vector<std::size_t> stream_sizes{640*480, 1280*480, 1280*720, 2560*720};
  // all bbp
  std::vector<std::size_t> bbps{1, 2, 3};

  for (auto&& ss : stream_sizes) {
    for (auto&& bbp : bbps) {
      sizes.insert(ss * bbp);
    }
  }

  g_data_caches.SetProperSizes(sizes);
  g_left_data_caches.SetProperSizes(sizes);
  g_right_data_caches.SetProperSizes(sizes);
  g_depth_data_caches.SetProperSizes(sizes);
}

DataCaches::data_ptr_t get_cache_fixed(const ImageType& type,
    const std::size_t& size) {
  if (type == ImageType::IMAGE_DEPTH) {
    return g_depth_data_caches.GetFixed(size);
  } else if (type == ImageType::IMAGE_LEFT_COLOR) {
    return g_left_data_caches.GetFixed(size);
  } else if (type == ImageType::IMAGE_RIGHT_COLOR) {
    return g_right_data_caches.GetFixed(size);
  } else {
    return g_data_caches.GetFixed(size);
  }
}

DataCaches::data_ptr_t get_cache_proper(const ImageType& type,
    const std::size_t& size) {
  if (type == ImageType::IMAGE_DEPTH) {
    return g_depth_data_caches.GetProper(size);
  } else if (type == ImageType::IMAGE_LEFT_COLOR) {
    return g_left_data_caches.GetProper(size);
  } else if (type == ImageType::IMAGE_RIGHT_COLOR) {
    return g_right_data_caches.GetProper(size);
  } else {
    return g_data_caches.GetProper(size);
  }
}

Image::pointer get_cache_image(const Image::pointer& image,
    const ImageFormat& format, int width, int height) {
  auto&& result = Image::Create(image->type(), format, width, height, false);
  result->set_frame_id(image->frame_id());
  result->set_is_dual(image->is_dual());
  return result;
}

Image::pointer get_cache_image(const Image::pointer& image,
    const ImageFormat& format) {
  return get_cache_image(image, format, image->width(), image->height());
}

}  // namespace

Image::Image(const ImageType& type, const ImageFormat& format,
    int width, int height, bool is_buffer)
  : type_(type),
    format_(format),
    width_(width),
    height_(height),
    is_buffer_(is_buffer),
    raw_format_(format),
    frame_id_(0),
    is_dual_(false) {
  static bool is_cache_proper_sizes_set = false;
  if (!is_cache_proper_sizes_set) {
    init_cache_proper_sizes();
    is_cache_proper_sizes_set = true;
  }
  auto&& n = get_image_size(format, width, height);
  data_ = get_cache_fixed(type_, n);
  set_valid_size(n);
}

Image::~Image() {
}

Image::pointer Image::Create(const ImageType& type, const ImageFormat& format,
    int width, int height, bool is_buffer) {
  switch (type) {
    case ImageType::IMAGE_LEFT_COLOR:
    case ImageType::IMAGE_RIGHT_COLOR:
      return ImageColor::Create(type, format, width, height, is_buffer);
    case ImageType::IMAGE_DEPTH:
      return ImageDepth::Create(format, width, height, is_buffer);
    default:
      throw new std::runtime_error("ImageType must be color or depth");
  }
}

void Image::set_valid_size(std::size_t valid_size) {
  if (valid_size > data_size()) {
    // resize data to valid size
    data_ = get_cache_proper(type_, valid_size);
  }
  valid_size_ = valid_size;
}

#ifdef WITH_OPENCV
cv::Mat Image::ToMat() {
  return cv::Mat(height_, width_, get_mat_type(format_), data());
}
#endif

Image::pointer Image::Clone() const {
  auto image = Create(type_, format_, width_, height_, false);
  image->set_frame_id(frame_id_);
  image->set_is_dual(is_dual_);
  image->set_valid_size(valid_size_);
  // The valid size of some compress format will much smaller, e.g. MJPG.
  // Therefore, we could only copy valid data to another.
  std::copy(data_->begin(), data_->begin() + valid_size_,
      image->data_->begin());
  return image;
}

Image::pointer Image::Shadow(const ImageType& type) const {
  auto image = Create(type, format_, width_, height_, false);
  image->set_frame_id(frame_id_);
  image->set_is_dual(is_dual_);
  image->set_valid_size(valid_size_);
  // Set data to this
  image->data_ = data_;
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

ImageColor::ImageColor(const ImageType& type, const ImageFormat& format,
    int width, int height, bool is_buffer)
  : Image(type, format, width, height, is_buffer) {
}

ImageColor::~ImageColor() {
}

ImageColor::pointer ImageColor::Create(const ImageType& type,
    const ImageFormat& format, int width, int height, bool is_buffer) {
  if (type == ImageType::IMAGE_LEFT_COLOR
      || type == ImageType::IMAGE_RIGHT_COLOR) {
    return pointer(new ImageColor(type, format, width, height, is_buffer));
  } else {
    throw new std::runtime_error("ImageType must be color");
  }
}

Image::pointer ImageColor::To(const ImageFormat& format) {
  // LOGI(strings::format_string("color src: %d, dst: %d", format_, format));
  if (format == format_) {
    return shared_from_this();
  }
  switch (format_) {  // src
    case ImageFormat::COLOR_BGR:
      if (is_dual_) goto to_fail;
      if (format == ImageFormat::COLOR_RGB) {
        BGR_TO_RGB(data(), width_, height_);
        format_ = format;
        return shared_from_this();
      }
      break;
    case ImageFormat::COLOR_RGB:
      if (is_dual_) goto to_fail;
      if (format == ImageFormat::COLOR_BGR) {
        RGB_TO_BGR(data(), width_, height_);
        format_ = format;
        return shared_from_this();
      }
      break;
    case ImageFormat::COLOR_YUYV:
      if (is_dual_) {
        /*
        Image::pointer color = shared_from_this();
        if (color->type() == ImageType::IMAGE_LEFT_COLOR) {
          color = images::split_left_color(color);
        } else if (color->type() == ImageType::IMAGE_RIGHT_COLOR) {
          color = images::split_right_color(color);
        } else {
          goto to_fail;
        }
        auto image = get_cache_image(shared_from_this(), format,
            width_ / 2, height_);
        image->set_is_dual(false);
        if (format == ImageFormat::COLOR_RGB) {
          YUYV_TO_RGB(color->data(), image->data(), width_ / 2, height_);
        } else if (format == ImageFormat::COLOR_BGR) {
          YUYV_TO_BGR(color->data(), image->data(), width_ / 2, height_);
        } else {
          goto to_fail;
        }
        return image;
        */
        auto image = get_cache_image(shared_from_this(), format,
            width_ / 2, height_);
        image->set_is_dual(false);
        if (format == ImageFormat::COLOR_RGB) {
          if (type_ == ImageType::IMAGE_LEFT_COLOR) {
            YUYV_TO_RGB_LEFT(data(), image->data(), width_, height_);
          } else if (type_ == ImageType::IMAGE_RIGHT_COLOR) {
            YUYV_TO_RGB_RIGHT(data(), image->data(), width_, height_);
          } else {
            goto to_fail;
          }
        } else if (format == ImageFormat::COLOR_BGR) {
          if (type_ == ImageType::IMAGE_LEFT_COLOR) {
            YUYV_TO_BGR_LEFT(data(), image->data(), width_, height_);
          } else if (type_ == ImageType::IMAGE_RIGHT_COLOR) {
            YUYV_TO_BGR_RIGHT(data(), image->data(), width_, height_);
          } else {
            goto to_fail;
          }
        } else {
          goto to_fail;
        }
        return image;
      } else {
        auto image = get_cache_image(shared_from_this(), format);
        if (format == ImageFormat::COLOR_RGB) {
          YUYV_TO_RGB(data(), image->data(), width_, height_);
        } else if (format == ImageFormat::COLOR_BGR) {
          YUYV_TO_BGR(data(), image->data(), width_, height_);
        } else {
          goto to_fail;
        }
        return image;
      }
      break;
    case ImageFormat::COLOR_MJPG:
      if (format == ImageFormat::COLOR_RGB) {
        auto image = get_cache_image(shared_from_this(),
            ImageFormat::COLOR_RGB);
        MJPEG_TO_RGB_LIBJPEG(data(), valid_size_, image->data());
        if (is_dual_) {
          auto half = get_cache_image(shared_from_this(),
              ImageFormat::COLOR_RGB, width_ / 2, height_);
          half->set_is_dual(false);
          if (type_ == ImageType::IMAGE_LEFT_COLOR) {
            RGB_TO_RGB_LEFT(image->data(), half->data(), width_, height_);
          } else if (type_ == ImageType::IMAGE_RIGHT_COLOR) {
            RGB_TO_RGB_RIGHT(image->data(), half->data(), width_, height_);
          } else {
            goto to_fail;
          }
          return half;  // left or right
        }
        return image;  // left only
      } else if (format == ImageFormat::COLOR_BGR) {
        // return To(ImageFormat::COLOR_RGB)->To(ImageFormat::COLOR_BGR);
        auto image = get_cache_image(shared_from_this(),
            ImageFormat::COLOR_RGB);
        MJPEG_TO_RGB_LIBJPEG(data(), valid_size_, image->data());
        if (is_dual_) {
          auto half = get_cache_image(shared_from_this(),
              ImageFormat::COLOR_BGR, width_ / 2, height_);
          half->set_is_dual(false);
          // Split from rgb to bgr directly
          if (type_ == ImageType::IMAGE_LEFT_COLOR) {
            RGB_TO_BGR_LEFT(image->data(), half->data(), width_, height_);
          } else if (type_ == ImageType::IMAGE_RIGHT_COLOR) {
            RGB_TO_BGR_RIGHT(image->data(), half->data(), width_, height_);
          } else {
            goto to_fail;
          }
          return half;  // left or right
        }
        return image->To(ImageFormat::COLOR_BGR);  // left only
      }
      break;
    default: break;
  }
to_fail:
  throw new std::runtime_error(strings::format_string(
      "Can not convert from %s to %s", format_, format));
}

// ImageDepth

ImageDepth::ImageDepth(const ImageFormat& format, int width, int height,
    bool is_buffer)
  : Image(ImageType::IMAGE_DEPTH, format, width, height, is_buffer) {
}

ImageDepth::~ImageDepth() {
}

Image::pointer ImageDepth::To(const ImageFormat& format) {
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

        auto image = get_cache_image(shared_from_this(), format);
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
