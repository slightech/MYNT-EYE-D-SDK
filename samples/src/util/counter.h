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
#ifndef MYNTEYE_SAMPLES_UTIL_COUNTER_H_
#define MYNTEYE_SAMPLES_UTIL_COUNTER_H_
#pragma once

#include <cstdint>
#include <queue>

#include "mynteye/util/times.h"

namespace mynteye {
namespace util {

class Counter {
 public:
  using clock = times::clock;

  explicit Counter(std::size_t fps_frame_count = 10)
    : count_(0), fps_(0), fps_frame_count_(fps_frame_count) {
    time_beg_ = times::now();
    if (fps_frame_count_ >= 2) {
      fps_frame_times_.push(time_beg_);
    }
  }

  double fps() const {
    // std::lock_guard<std::mutex> _(mtx_);
    return fps_;
  }

  std::uint64_t count() const {
    // std::lock_guard<std::mutex> _(mtx_);
    return count_;
  }

  void Update() {
    // std::lock_guard<std::mutex> _(mtx_);
    ++count_;

    auto time_now = times::now();
    if (fps_frame_count_ >= 2) {
      if (fps_frame_times_.size() == fps_frame_count_) {
        fps_frame_times_.pop();
      }
      fps_frame_times_.push(time_now);

      auto elapsed = times::count<times::milliseconds>(
          fps_frame_times_.back() - fps_frame_times_.front());
      fps_ = 1000.f * (fps_frame_times_.size() - 1) / elapsed;
    } else {
      auto elapsed = times::count<times::milliseconds>(time_now - time_beg_);
      fps_ = 1000.f * count_ / elapsed;
    }
  }

 private:
  clock::time_point time_beg_;
  std::uint64_t count_;
  double fps_;

  std::queue<clock::time_point> fps_frame_times_;
  std::size_t fps_frame_count_;

  // std::mutex mtx_;
};

}  // namespace util
}  // namespace mynteye

#endif  // MYNTEYE_SAMPLES_UTIL_COUNTER_H_
