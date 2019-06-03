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
#include "mynteyed/internal/streams.h"

#include "mynteyed/device/device.h"
#include "mynteyed/util/log.h"
#include "mynteyed/util/rate.h"
#include "mynteyed/util/strings.h"
#include "mynteyed/util/times.h"
#include "mynteyed/internal/match.h"

// set 1 only for the latest stream data
#define STREAM_DATAS_MAX_SIZE 4
#define IMG_INFO_QUEUE_MAX_SIZE 120  // 60fps, 2s
#define IMG_INFO_SYNC_FREQUENCY 100  // 100hz

MYNTEYE_USE_NAMESPACE

Streams::Streams(std::shared_ptr<Device> device)
  : device_(device),
    all_image_types_({
      ImageType::IMAGE_LEFT_COLOR,
      ImageType::IMAGE_RIGHT_COLOR,
      ImageType::IMAGE_DEPTH}),
    all_stream_types_({STREAM_COLOR, STREAM_DEPTH}),
    is_image_info_enabled_(false),
    is_image_info_sync_(false),
    is_right_color_supported_(false),
    stream_datas_max_size_(STREAM_DATAS_MAX_SIZE),
    is_stream_capturing_(false),
    stream_queue_map_({
      {STREAM_COLOR, std::make_shared<stream_queue_t>(stream_datas_max_size_)},
      {STREAM_DEPTH, std::make_shared<stream_queue_t>(stream_datas_max_size_)}
    }),
    stream_info_queue_map_({
      {STREAM_COLOR,
          std::make_shared<img_info_queue_t>(IMG_INFO_QUEUE_MAX_SIZE)},
      {STREAM_DEPTH,
          std::make_shared<img_info_queue_t>(IMG_INFO_QUEUE_MAX_SIZE)},
    }),
    img_info_callback_(nullptr),
    img_data_callbacks_({
      {ImageType::IMAGE_LEFT_COLOR, nullptr},
      {ImageType::IMAGE_RIGHT_COLOR, nullptr},
      {ImageType::IMAGE_DEPTH, nullptr}}) {

    match_.reset(new Match());
}

Streams::~Streams() {
}

void Streams::EnableImageInfo(bool sync) {
  if (is_image_info_enabled_ && is_image_info_sync_ == sync) {
    return;
  }
  OnImageInfoStateChanged(true, sync);
}

void Streams::DisableImageInfo() {
  if (!is_image_info_enabled_ && !is_image_info_sync_) {
    return;
  }
  OnImageInfoStateChanged(false, false);
}

bool Streams::IsImageInfoEnabled() const {
  return is_image_info_enabled_;
}

bool Streams::IsImageInfoSynced() const {
  return is_image_info_sync_;
}

void Streams::SetImgInfoCallback(img_info_callback_t callback) {
  img_info_callback_ = callback;
}

void Streams::EnableStreamData(const ImageType& type) {
  if (IsStreamDataEnabled(type)) return;
  switch (type) {
    case ImageType::IMAGE_LEFT_COLOR:
    case ImageType::IMAGE_RIGHT_COLOR:
      OnStreamDataStateChanged(type, true);
      break;
    case ImageType::IMAGE_DEPTH:
      OnStreamDataStateChanged(type, true);
      break;
    case ImageType::IMAGE_ALL:
      EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
      EnableStreamData(ImageType::IMAGE_RIGHT_COLOR);
      EnableStreamData(ImageType::IMAGE_DEPTH);
      break;
  }
}

void Streams::DisableStreamData(const ImageType& type) {
  if (!IsStreamDataEnabled(type)) return;
  switch (type) {
    case ImageType::IMAGE_LEFT_COLOR:
    case ImageType::IMAGE_RIGHT_COLOR:
    case ImageType::IMAGE_DEPTH:
      OnStreamDataStateChanged(type, false);
      break;
    case ImageType::IMAGE_ALL:
      DisableStreamData(ImageType::IMAGE_LEFT_COLOR);
      DisableStreamData(ImageType::IMAGE_RIGHT_COLOR);
      DisableStreamData(ImageType::IMAGE_DEPTH);
      break;
  }
}

bool Streams::IsStreamDataEnabled(const ImageType& type) const {
  return is_image_enabled_set_.find(type) != is_image_enabled_set_.end();
}

bool Streams::HasStreamDataEnabled() const {
  return !is_image_enabled_set_.empty();
}

void Streams::EnableStreamDatas(std::size_t max_size) {
  std::size_t size = max_size;
  if (size < 1) size = 1;
  if (stream_datas_max_size_ == size) {
    return;
  }
  stream_datas_max_size_ = size;

  // Limit the stream queue
  for (auto&& type : all_stream_types_) {
    stream_queue_map_[type] = std::make_shared<stream_queue_t>(size);
  }
}

Streams::img_data_t Streams::GetStreamData(const ImageType& type) {
  auto &&datas = GetStreamDatas(type);
  if (datas.empty()) return {};
  return std::move(datas.back());
}

Streams::img_datas_t Streams::GetStreamDatas(const ImageType& type) {
  device_->CheckOpened(__func__);

  if (!IsStreamDataEnabled(type)) {
    LOGW(strings::format_string("Warning: Please enable %s, before you wanna "
        "get them. However, we will try enable it now and there is not data "
        "return this time.", strings::to_string(type)));
    EnableStreamData(type);
    return {};
  }

  return match_->GetStreamDatas(type);
}

void Streams::SetStreamCallback(const ImageType& type,
    img_data_callback_t callback) {
  img_data_callbacks_[type] = callback;
}

void Streams::OnCameraOpen() {
  is_right_color_supported_ = device_->IsRightColorSupported();
  match_->InitStreamKey(device_->DepthDeviceOpened());
  StartStreamCapturing();
}

void Streams::OnCameraClose() {
  StopStreamCapturing();
}

void Streams::OnImageInfoCallback(const ImgInfoPacket& packet) {
  auto&& img_info = std::make_shared<ImgInfo>();

  img_info->frame_id = packet.frame_id;
  img_info->timestamp = packet.timestamp;
  img_info->exposure_time = packet.exposure_time;

  if (is_image_info_sync_) {
    // push info
    for (auto&& info : stream_info_queue_map_) {
      info.second->Put(img_info);
    }
    SyncStreamWithInfo(false);
  }

  // callback
  if (img_info_callback_) {
    img_info_callback_(img_info);
  }
}

Streams::StreamType Streams::GetStreamType(const ImageType& type) const {
  if (IsStreamColor(type)) return STREAM_COLOR;
  if (IsStreamDepth(type)) return STREAM_DEPTH;
  throw_error("Image type is unaccepted to be a stream type.");
}

bool Streams::IsStreamEnabled(const StreamType& type) const {
  if (type == STREAM_COLOR) {
    return IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR)
        || IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR);
  } else if (type == STREAM_DEPTH) {
    return IsStreamDataEnabled(ImageType::IMAGE_DEPTH);
  }
  return false;
}

void Streams::StartStreamCapturing() {
  if (is_stream_capturing_) return;
  if (!HasStreamDataEnabled()) return;
  if (!device_->IsOpened()) return;

  if (IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR)
      && !is_right_color_supported_) {
    throw_error("If wanna get right color, must use one of these stream mode:"
        "\n  * STREAM_1280x480\n  * STREAM_2560x720"
        "\nOr cancel EnableStreamData(ImageType::IMAGE_RIGHT_COLOR)");
  }

  match_->SetIRDepthStatus(IsIRDepthOnly());
  is_stream_capturing_ = true;
  stream_capture_thread_ = std::thread([this]() {
    // Rate rate(device_->GetOpenParams().framerate);
    Rate rate(100);
    while (is_stream_capturing_) {
      CaptureStreamColor();
      CaptureStreamDepth();
      SyncStreamWithInfo(true);
      rate.Sleep();
    }
  });
}

void Streams::StopStreamCapturing() {
  if (!is_stream_capturing_) return;
  is_stream_capturing_ = false;
  if (stream_capture_thread_.joinable()) {
    stream_capture_thread_.join();
  }
}

void Streams::OnImageInfoStateChanged(bool enabled, bool sync) {
  is_image_info_enabled_ = enabled;
  is_image_info_sync_ = sync;
  if (!sync) {
    // clear queue for sync
    for (auto&& datas : stream_queue_map_) {
      datas.second->Clear();
    }
    for (auto&& infos : stream_info_queue_map_) {
      infos.second->Clear();
    }
  }
}

void Streams::OnStreamDataStateChanged(const ImageType& type, bool enabled) {
  if (enabled) {
    is_image_enabled_set_.insert(type);
    StartStreamCapturing();
  } else {
    is_image_enabled_set_.erase(type);
    if (!HasStreamDataEnabled()) {
      StopStreamCapturing();
    }
    // clear queue
    auto&& stream_type = GetStreamType(type);
    stream_queue_map_[stream_type]->Clear();
    stream_info_queue_map_[stream_type]->Clear();
  }
}

void Streams::SyncStreamWithInfo(bool force) {
  if (!is_image_info_sync_) return;

  // keep sync frequency
  {
    using clock = times::clock;
    static clock::duration time_dist{clock::period::den / clock::period::num \
        / IMG_INFO_SYNC_FREQUENCY};
    static clock::time_point time_prev = times::now();

    if (force) {
      time_prev = times::now();
    } else {
      auto time_now = times::now();
      if (time_now - time_prev < time_dist) {
        time_prev = time_now;
        // LOGI("Skip sync image info");
        return;
      }
      time_prev = time_now;
    }
  }

  for (auto&& type : all_stream_types_) {
    if (!IsStreamEnabled(type)) continue;

    auto&& streams = stream_queue_map_[type];
    if (streams->empty()) continue;

    auto&& infos = stream_info_queue_map_[type];
    if (infos->empty()) continue;

    std::lock_guard<std::mutex> _(streams->mutex());
    std::lock_guard<std::mutex> _i(infos->mutex());

    if (streams->empty() || infos->empty()) continue;

    bool next = false;
    for (auto st_it = streams->begin(); st_it != streams->end();) {
      next = true;
      auto frame_id = (*st_it)->frame_id();

      for (auto info_it = infos->begin();
          info_it != infos->end();) {
        if (frame_id == (*info_it)->frame_id) {
          OnStreamSyncedInfoCaptured(type, *st_it, *info_it);

          st_it = streams->erase(st_it);
          info_it = infos->erase(info_it);
          next = false;
          break;
        }

        ++info_it;
      }
      if (next) ++st_it;
    }
  }
}

void Streams::OnStreamSyncedInfoCaptured(const StreamType& type,
    const Image::pointer& stream,
    const img_info_ptr_t& stream_info) {
  if (type == StreamType::STREAM_COLOR) {
    DoImageColorCaptured(stream, stream_info);
  } else if (type == StreamType::STREAM_DEPTH) {
    DoImageDepthCaptured(stream, stream_info);
  } else {
    throw_error("Unbelievable stream type");
  }
}

void Streams::CaptureStreamColor() {
  if (!IsStreamEnabled(STREAM_COLOR)) return;

  auto color = device_->GetImageColor();
  if (!color) { return; }
  // LOGI("%s: %d", __func__, color->frame_id());

  color->set_is_dual(is_right_color_supported_);

  // Ensure not buffer to user, as it may changed when captured again.
  if (color->is_buffer()) {
    color = color->Clone();
  }

  if (is_image_info_sync_) {
    stream_queue_map_[STREAM_COLOR]->Put(color);
  } else {
    DoImageColorCaptured(color, nullptr);
  }
}

void Streams::CaptureStreamDepth() {
  if (!IsStreamEnabled(STREAM_DEPTH)) return;

  auto depth = device_->GetImageDepth();
  if (!depth) { return; }
  // LOGI("%s: %d", __func__, depth->frame_id());

  // Ensure not buffer to user, as it may changed when captured again.
  if (depth->is_buffer()) {
    depth = depth->Clone();
  }

  // On win, could not sync image info for depth
  if (is_image_info_sync_) {
    stream_queue_map_[STREAM_DEPTH]->Put(depth);
  } else {
    DoImageDepthCaptured(depth, nullptr);
  }
}

void Streams::DoImageColorCaptured(const Image::pointer& color,
    const img_info_ptr_t& info) {
  if (color->is_dual()) {
    // left, right may only one or both enabled
    if (IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR)) {
      DoStreamDataCaptured(color->Shadow(ImageType::IMAGE_LEFT_COLOR), info);
    }
    if (IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR)) {
      DoStreamDataCaptured(color->Shadow(ImageType::IMAGE_RIGHT_COLOR), info);
    }
  } else /*if (left_enabled)*/ {
    // left must enabled if left only, as could not enable right if left only
    DoStreamDataCaptured(color, info);
  }
}

void Streams::DoImageDepthCaptured(const Image::pointer& depth,
    const img_info_ptr_t& info) {
  DoStreamDataCaptured(depth, info);
}

void Streams::DoStreamDataCaptured(const Image::pointer& image,
    const img_info_ptr_t& info) {
  auto&& type = image->type();
  StreamData data{image, info};
  NotifyStreamData(type, data);
  if (img_data_callbacks_[type]) {
    img_data_callbacks_[type](data);
  }
}

void Streams::NotifyStreamData(const ImageType &type,
    const StreamData &data) {
  if (match_) {
    match_->OnStreamDataCallback(type, data);
  }
}

bool Streams::IsIRDepthOnly() {
  return device_->IsIRDepthOnly();
}

bool Streams::WaitForStreamData() {
  return match_->WaitForStreamData();
}
