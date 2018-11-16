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
#include "mynteye/util/rate.h"
#include "mynteye/util/strings.h"

#define IMAGE_QUEUE_MAX_SIZE 4
#define IMAGE_QUEUE_WITH_INFO_MAX_SIZE 4
#define IMAGE_INFO_QUEUE_MAX_SIZE 120  // 60fps, 2s

MYNTEYE_USE_NAMESPACE

Streams::Streams(std::shared_ptr<Device> device)
  : device_(device),
    all_image_types_({
      ImageType::IMAGE_LEFT_COLOR,
      ImageType::IMAGE_RIGHT_COLOR,
      ImageType::IMAGE_DEPTH}),
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
  InitImageWithInfoQueue();
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
        image_queue_map_[type] =
            std::make_shared<image_queue_t>(IMAGE_QUEUE_MAX_SIZE);
      }
      break;
    case ImageType::IMAGE_ALL:
      EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
      EnableStreamData(ImageType::IMAGE_RIGHT_COLOR);
      EnableStreamData(ImageType::IMAGE_DEPTH);
      break;
  }
  InitImageWithInfoQueue();
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
  if (datas.empty()) return {};
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

  if (is_image_info_sync_) {
    auto&& datas = image_with_info_datas_map_[type]->TakeAll();
    return {datas.begin(), datas.end()};
  } else {
    auto&& images = image_queue_map_[type]->TakeAll();

    datas_t datas;
    while (!images.empty()) {
      datas.push_back({images.front(), nullptr});
      images.pop_front();
    }
    return std::move(datas);
  }
}

void Streams::OnCameraOpen() {
  is_right_color_supported_ = IsRightColorSupported();
  StartImageCapturing();
}

void Streams::OnCameraClose() {
  StopImageCapturing();
}

void Streams::OnImageInfoCallback(const ImgInfoPacket &packet) {
  auto&& img_info = std::make_shared<ImgInfo>();

  img_info->frame_id = packet.frame_id;
  img_info->timestamp = packet.timestamp;
  img_info->exposure_time = packet.exposure_time;

  // push info
  for (auto&& info : image_info_queue_map_) {
    info.second->Put(img_info);
  }

  SyncImageWithInfo();

  // callback
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
    Rate rate(device_->GetOpenParams().framerate);
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
      SyncImageWithInfo();
      rate.Sleep();
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

void Streams::InitImageWithInfoQueue() {
  if (!is_image_info_sync_) return;

  // init queue for enabled image type
  auto&& img_datas_map = image_with_info_datas_map_;
  auto&& img_info_queue_map = image_info_queue_map_;
  for (auto&& type : all_image_types_) {
    if (!IsStreamDataEnabled(type)) continue;

    // init image with info vector
    if (img_datas_map.find(type) == img_datas_map.end()) {
      img_datas_map[type] =
          std::make_shared<img_datas_t>(IMAGE_QUEUE_WITH_INFO_MAX_SIZE);
    }

    // init image info queue
    if (img_info_queue_map.find(type) == img_info_queue_map.end()) {
      img_info_queue_map[type] =
          std::make_shared<img_info_queue_t>(IMAGE_INFO_QUEUE_MAX_SIZE);
    }
  }
}

void Streams::SyncImageWithInfo() {
  if (!is_image_info_sync_) return;

  // ...

  for (auto&& type : all_image_types_) {
    if (!IsStreamDataEnabled(type)) continue;

    auto&& imgs = image_queue_map_[type];
    if (imgs->empty()) continue;

    auto&& img_infos = image_info_queue_map_[type];
    if (img_infos->empty()) continue;

    std::lock_guard<std::mutex> _(imgs->mutex());
    std::lock_guard<std::mutex> _i(img_infos->mutex());

    if (imgs->empty() || img_infos->empty()) continue;

    bool next = false;
    for (auto img_it = imgs->begin(); img_it != imgs->end();) {
      next = true;
      auto frame_id = (*img_it)->frame_id();

      for (auto img_info_it = img_infos->begin();
          img_info_it != img_infos->end();) {
        if (frame_id == (*img_info_it)->frame_id) {
          PushImageWithInfo(*img_it, *img_info_it);

          img_it = imgs->erase(img_it);
          img_info_it = img_infos->erase(img_info_it);
          next = false;
          break;
        }

        ++img_info_it;
      }
      if (next) ++img_it;
    }
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
  // LOGI("%s: %d", __func__, color->frame_id());
  PushImage(color);
}

void Streams::OnRightColorCaptured(const Image::pointer& color) {
  // LOGI("%s: %d", __func__, color->frame_id());
  PushImage(color);
}

void Streams::OnDepthCaptured(const Image::pointer& depth) {
  // LOGI("%s: %d", __func__, depth->frame_id());
  PushImage(depth);
}

void Streams::PushImage(const Image::pointer& image) {
  auto type = image->type();
  if (image->is_buffer()) {
    image_queue_map_[type]->Put(image->Clone());
  } else {
    image_queue_map_[type]->Put(image);
  }
}

void Streams::PushImageWithInfo(const Image::pointer& image,
    const img_info_ptr_t& info) {
  auto type = image->type();
  auto&& images = image_with_info_datas_map_[type];
  if (image->is_buffer()) {
    images->Put({image->Clone(), info});
  } else {
    images->Put({image, info});
  }
}
