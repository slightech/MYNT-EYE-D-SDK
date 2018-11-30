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
#ifndef MYNTEYE_DEVICE_IMAGE_H_
#define MYNTEYE_DEVICE_IMAGE_H_
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <vector>
#include <iostream>

#ifdef WITH_OPENCV
#include <opencv2/core/core.hpp>
#endif

#include "mynteyed/device/types.h"
#include "mynteyed/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

class MYNTEYE_API Image {
 public:
  using pointer = std::shared_ptr<Image>;

  using data_t = std::vector<std::uint8_t>;
  using data_ptr_t = std::shared_ptr<data_t>;

 protected:
  Image(const ImageType& type, const ImageFormat& format,
      int width, int height, bool is_buffer);

 public:
  virtual ~Image();

  static pointer Create(const ImageType& type, const ImageFormat& format,
      int width, int height, bool is_buffer);

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

  std::size_t size() const {
    return width_ * height_;
  }

  bool is_buffer() const {
    return is_buffer_;
  }

  int frame_id() const {
    return frame_id_;
  }

  void set_frame_id(int frame_id) {
    frame_id_ = frame_id;
  }

  bool is_dual() const {
    return is_dual_;
  }

  void set_is_dual(bool is_dual) {
    is_dual_ = is_dual;
  }

  std::uint8_t* data() {
    return data_->data();
  }

  const std::uint8_t* data() const {
    return data_->data();
  }

  std::size_t data_size() const {
    return data_->size();
  }

  std::size_t valid_size() const {
    return valid_size_;
  }

  // will resize data if larger then data size
  void set_valid_size(std::size_t valid_size);

  virtual pointer To(const ImageFormat& format) = 0;

#ifdef WITH_OPENCV
  cv::Mat ToMat();
#endif

  pointer Clone() const;
  pointer Shadow(const ImageType& type) const;

  bool ResetBuffer();

 protected:
  ImageType type_;
  ImageFormat format_;
  int width_;
  int height_;
  bool is_buffer_;

  ImageFormat raw_format_;

  // Frame id
  int frame_id_;
  // Special state for dual data
  bool is_dual_;

  data_ptr_t data_;
  // The real valid size of some compress format or other cases.
  std::size_t valid_size_;

  MYNTEYE_DISABLE_COPY(Image)
  MYNTEYE_DISABLE_MOVE(Image)
};

class MYNTEYE_API ImageColor : public Image,
    public std::enable_shared_from_this<ImageColor> {
 public:
  using pointer = std::shared_ptr<ImageColor>;

 protected:
  ImageColor(const ImageType& type, const ImageFormat& format,
      int width, int height, bool is_buffer);

 public:
  virtual ~ImageColor();

  static pointer Create(const ImageFormat& format, int width, int height,
      bool is_buffer) {
    return Create(ImageType::IMAGE_LEFT_COLOR, format, width, height,
        is_buffer);
  }

  static pointer Create(const ImageType& type, const ImageFormat& format,
      int width, int height, bool is_buffer);

  Image::pointer To(const ImageFormat& format) override;

 private:
  MYNTEYE_DISABLE_COPY(ImageColor)
  MYNTEYE_DISABLE_MOVE(ImageColor)
};

class MYNTEYE_API ImageDepth : public Image,
    public std::enable_shared_from_this<ImageDepth> {
 public:
  using pointer = std::shared_ptr<ImageDepth>;

 protected:
  ImageDepth(const ImageFormat& format, int width, int height, bool is_buffer);

 public:
  virtual ~ImageDepth();

  static pointer Create(const ImageFormat& format, int width, int height,
      bool is_buffer) {
    return pointer(new ImageDepth(format, width, height, is_buffer));
  }

  Image::pointer To(const ImageFormat& format) override;

 private:
  MYNTEYE_DISABLE_COPY(ImageDepth)
  MYNTEYE_DISABLE_MOVE(ImageDepth)
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_IMAGE_H_
