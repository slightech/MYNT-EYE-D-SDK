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
#ifndef MYNTEYE_INTERNAL_STREAMS_H_
#define MYNTEYE_INTERNAL_STREAMS_H_
#pragma once

// #include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <set>
#include <thread>
#include <vector>

#include "mynteyed/data/types_internal.h"
#include "mynteyed/internal/blocking_queue.h"
#include "mynteyed/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;
class Match;

class Streams {
 public:
  template <typename T>
  using queue_t = BlockingQueue<T, std::deque<T>>;

  using img_data_t = StreamData;
  using img_datas_t = std::vector<img_data_t>;
  using img_info_ptr_t = std::shared_ptr<ImgInfo>;

  // img infos
  using img_info_queue_t = queue_t<img_info_ptr_t>;
  using img_info_queue_ptr_t = std::shared_ptr<img_info_queue_t>;

  // img data callback
  using img_data_callback_t = std::function<void(const img_data_t& data)>;
  // img info callback
  using img_info_callback_t = std::function<void(const img_info_ptr_t& info)>;

  // stream types
  typedef enum StreamType {
    STREAM_COLOR,  // left or left+right
    STREAM_DEPTH,  // depth
  } stream_type_t;

  // stream queue
  using stream_queue_t = queue_t<Image::pointer>;
  using stream_queue_ptr_t = std::shared_ptr<stream_queue_t>;

  explicit Streams(std::shared_ptr<Device> device);
  ~Streams();

  /**
   * Enable image infos.
   *
   * If sync is false, indicates only can get infos from callback.
   * If sync is true, indicates can get infos from callback or access it from StreamData.
   */
  void EnableImageInfo(bool sync);
  void DisableImageInfo();
  bool IsImageInfoEnabled() const;
  bool IsImageInfoSynced() const;

  void SetImgInfoCallback(img_info_callback_t callback);

  void EnableStreamData(const ImageType& type);
  void DisableStreamData(const ImageType& type);
  bool IsStreamDataEnabled(const ImageType& type) const;
  bool HasStreamDataEnabled() const;

  /**
   * Enable stream datas.
   *
   * If max_size <= 1, indicates only can get the latest stream data using GetStreamDatas().
   * If max_size > 1, indicates can get more stream datas using GetStreamDatas().
   */
  void EnableStreamDatas(std::size_t max_size);
  bool IsStreamDatasEnabled() {
    return stream_datas_max_size_ > 1;
  }

  img_data_t GetStreamData(const ImageType& type);
  img_datas_t GetStreamDatas(const ImageType& type);

  void SetStreamCallback(const ImageType& type, img_data_callback_t callback);

  void OnCameraOpen();
  void OnCameraClose();

  void OnImageInfoCallback(const ImgInfoPacket& packet);

  bool IsIRDepthOnly();

  bool WaitForStreamData();

 private:
  void NotifyStreamData(const ImageType &type, const StreamData &data);

  bool IsStreamColor(const ImageType& type) const {
    return type == ImageType::IMAGE_LEFT_COLOR
        || type == ImageType::IMAGE_RIGHT_COLOR;
  }

  bool IsStreamDepth(const ImageType& type) const {
    return type == ImageType::IMAGE_DEPTH;
  }

  StreamType GetStreamType(const ImageType& type) const;

  bool IsStreamEnabled(const StreamType& type) const;

  void StartStreamCapturing();
  void StopStreamCapturing();

  void OnImageInfoStateChanged(bool enabled, bool sync);
  void OnStreamDataStateChanged(const ImageType& type, bool enabled);

  void SyncStreamWithInfo(bool force);
  void OnStreamSyncedInfoCaptured(const StreamType& type,
      const Image::pointer& stream,
      const img_info_ptr_t& stream_info);

  void CaptureStreamColor();
  void CaptureStreamDepth();

  void DoImageColorCaptured(const Image::pointer& color,
      const img_info_ptr_t& info);
  void DoImageDepthCaptured(const Image::pointer& depth,
      const img_info_ptr_t& info);

  void DoStreamDataCaptured(const Image::pointer& image,
      const img_info_ptr_t& info);

  std::shared_ptr<Device> device_;

  std::vector<ImageType> all_image_types_;
  std::vector<stream_type_t> all_stream_types_;

  bool is_image_info_enabled_;
  bool is_image_info_sync_;

  bool is_right_color_supported_;

  std::set<ImageType> is_image_enabled_set_;

  std::size_t stream_datas_max_size_;

  bool is_stream_capturing_;
  std::thread stream_capture_thread_;

  // stream queue, only for sync
  std::map<stream_type_t, stream_queue_ptr_t> stream_queue_map_;
  // stream info queue, only for sync
  std::map<stream_type_t, img_info_queue_ptr_t> stream_info_queue_map_;

  img_info_callback_t img_info_callback_;
  std::map<ImageType, img_data_callback_t> img_data_callbacks_;

  std::shared_ptr<Match> match_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_STREAMS_H_
