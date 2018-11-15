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
#include "mynteye/internal/streams.h"

#include "mynteye/device/device.h"
#include "mynteye/internal/image_utils.h"
#include "mynteye/util/log.h"
#include "mynteye/util/strings.h"

MYNTEYE_USE_NAMESPACE

Streams::Streams(std::shared_ptr<Device> device)
  : device_(device),
    is_image_info_enabled_(false),
    is_image_info_sync_(false),
    is_right_color_supported_(false),
    is_image_capturing_(false) {
}

Streams::~Streams() {
}

void Streams::EnableImageInfo(bool sync) {
  is_image_info_enabled_ = true;
  is_image_info_sync_ = sync;
}

bool Streams::IsImageInfoEnabled() {
  return is_image_info_enabled_;
}

void Streams::EnableStreamData(const ImageType& type) {
  switch (type) {
    case ImageType::IMAGE_LEFT_COLOR:
    case ImageType::IMAGE_RIGHT_COLOR:
    case ImageType::IMAGE_DEPTH:
      if (image_queue_map_.find(type) == image_queue_map_.end()) {
        image_queue_map_[type] = std::make_shared<image_queue_t>(4);
      }
      break;
    case ImageType::ALL:
      EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
      EnableStreamData(ImageType::IMAGE_RIGHT_COLOR);
      EnableStreamData(ImageType::IMAGE_DEPTH);
      break;
  }
  StartImageCapturing();
}

bool Streams::IsStreamDataEnabled(const ImageType& type) {
  return image_queue_map_.find(type) != image_queue_map_.end();
}

bool Streams::HasStreamDataEnabled() {
  return !image_queue_map_.empty();
}

Streams::data_t Streams::GetStreamData(const ImageType& type) {
  auto&& datas = GetStreamDatas(type);
  return std::move(datas.back());
}

Streams::datas_t Streams::GetStreamDatas(const ImageType& type) {
  device_->CheckOpened(__func__);

  if (!IsStreamDataEnabled(type)) {
    LOGW(strings::format_string("Warning: Please enable %s, before you wanna "
        "get them. However, we will try enable it now and there is not data "
        "return this time.", strings::to_string(type)));
    EnableStreamData(type);
    return {};
  }

  auto&& images = image_queue_map_[type]->TakeAll();
  datas_t datas;
  while (!images.empty()) {
    datas.push_back({images.front(), nullptr});
    images.pop();
  }
  return std::move(datas);
}

void Streams::OnCameraOpen() {
  is_right_color_supported_ = IsRightColorSupported();
  StartImageCapturing();
}

void Streams::OnCameraClose() {
  StopImageCapturing();
}

void Streams::OnImageInfoCallback(const ImgInfoPacket &packet) {
}

bool Streams::IsRightColorSupported() {
  device_->CheckOpened(__func__);
  auto stream_mode = device_->GetOpenParams().stream_mode;
  return stream_mode == StreamMode::STREAM_1280x480
      || stream_mode == StreamMode::STREAM_2560x720;
}

void Streams::StartImageCapturing() {
  if (is_image_capturing_) return;
  if (!HasStreamDataEnabled()) return;
  if (!device_->IsOpened()) return;

  if (IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR)
      && !is_right_color_supported_) {
    throw_error("If wanna get right color, must use one of these stream mode:"
        "\n  * STREAM_1280x480\n  * STREAM_2560x720"
        "\nOr cancel EnableStreamData(ImageType::IMAGE_RIGHT_COLOR)");
  }

  is_image_capturing_ = true;
  image_capture_thread_ = std::thread([this]() {
    while (is_image_capturing_) {
      if (IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR)
          || IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR)) {
        auto color = device_->GetImageColor();
        if (color) OnColorCaptured(color);
      }
      if (IsStreamDataEnabled(ImageType::IMAGE_DEPTH)) {
        auto depth = device_->GetImageDepth();
        if (depth) OnDepthCaptured(depth);
      }
    }
  });
}

void Streams::StopImageCapturing() {
  if (!is_image_capturing_) return;
  is_image_capturing_ = false;
  if (image_capture_thread_.joinable()) {
    image_capture_thread_.join();
  }
}

void Streams::OnColorCaptured(const Image::pointer& color) {
  if (is_right_color_supported_) {
    OnLeftColorCaptured(images::split_left_color(color));
    OnRightColorCaptured(images::split_right_color(color));
  } else {
    OnLeftColorCaptured(color);
  }
}

void Streams::OnLeftColorCaptured(const Image::pointer& color) {
  PushImage(color);
}

void Streams::OnRightColorCaptured(const Image::pointer& color) {
  PushImage(color);
}

void Streams::OnDepthCaptured(const Image::pointer& depth) {
  PushImage(depth);
}

void Streams::PushImage(const Image::pointer& color) {
  auto type = color->type();
  if (color->is_buffer()) {
    image_queue_map_[type]->Put(color->Clone());
  } else {
    image_queue_map_[type]->Put(color);
  }
}
