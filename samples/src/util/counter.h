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

#include <opencv2/core/core.hpp>

namespace mynteye {
namespace util {

class Counter {
public:
  Counter() : tick_beg_(0), fps_(0), count_(0) {
    tick_beg_ = static_cast<double>(cv::getTickCount());
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
    double tick_count = static_cast<double>(cv::getTickCount()) - tick_beg_;
    double elapsed = tick_count / cv::getTickFrequency();
    ++count_;
    fps_ = count_ / elapsed;
  }

private:
  double tick_beg_;
  double fps_;
  std::uint64_t count_;
  // std::mutex mtx_;
};

}  // namespace util
}  // namespace mynteye

#endif  // MYNTEYE_SAMPLES_UTIL_COUNTER_H_
