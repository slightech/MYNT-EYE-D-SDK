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
#ifndef MYNTEYE_IMAGE_H_
#define MYNTEYE_IMAGE_H_
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include <opencv2/core/core.hpp>

#include "mynteye/stubs/global.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Image {
 public:
  using pointer = std::shared_ptr<Image>;

 protected:
  Image(ImageType type, ImageFormat format, int width, int height,
      bool is_buffer);

 public:
  virtual ~Image();

  static pointer Create(ImageType type, ImageFormat format, int width,
      int height, bool is_buffer);

  ImageType type() const {
    return type_;
  }

  ImageFormat format() const {
    return format_;
  }

  int width() const {
    return width_;
  }

  int height() const {
    return height_;
  }

  bool is_buffer() const {
    return is_buffer_;
  }

  std::uint8_t* data() {
    return data_.data();
  }

  const std::uint8_t* data() const {
    return data_.data();
  }

  std::size_t size() const {
    return data_.size();
  }

  std::size_t valid_size() const {
    return valid_size_;
  }

  void set_valid_size(std::size_t valid_size) {
    valid_size_ = valid_size;
  }

  virtual pointer To(ImageFormat format) = 0;

  cv::Mat ToMat();

  pointer Clone() const;

 protected:
  Image::pointer GetCache(const ImageFormat& format);

  ImageType type_;
  ImageFormat format_;
  int width_;
  int height_;
  bool is_buffer_;

  std::vector<std::uint8_t> data_;
  std::size_t valid_size_;

  std::map<int, Image::pointer> bpp_caches_;

  MYNTEYE_DISABLE_COPY(Image)
  MYNTEYE_DISABLE_MOVE(Image)
};

class ImageColor : public Image,
    public std::enable_shared_from_this<ImageColor> {
 public:
  using pointer = std::shared_ptr<ImageColor>;

 protected:
  ImageColor(ImageFormat format, int width, int height, bool is_buffer);

 public:
  virtual ~ImageColor();

  static pointer Create(ImageFormat format, int width, int height,
      bool is_buffer) {
    return pointer(new ImageColor(format, width, height, is_buffer));
  }

  Image::pointer To(ImageFormat format) override;

 private:
  MYNTEYE_DISABLE_COPY(ImageColor)
  MYNTEYE_DISABLE_MOVE(ImageColor)
};

class ImageDepth : public Image,
    public std::enable_shared_from_this<ImageDepth> {
 public:
  using pointer = std::shared_ptr<ImageDepth>;

 protected:
  ImageDepth(ImageFormat format, int width, int height, bool is_buffer);

 public:
  virtual ~ImageDepth();

  static pointer Create(ImageFormat format, int width, int height,
      bool is_buffer) {
    return pointer(new ImageDepth(format, width, height, is_buffer));
  }

  Image::pointer To(ImageFormat format) override;

 private:
  MYNTEYE_DISABLE_COPY(ImageDepth)
  MYNTEYE_DISABLE_MOVE(ImageDepth)
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_IMAGE_H_
