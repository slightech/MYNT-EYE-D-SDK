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
#include <iostream>
#include <queue>
#include <string>
#include <sstream>

#include "mynteyed/util/times.h"

MYNTEYE_BEGIN_NAMESPACE

namespace util {

class Counter {
 public:
  using clock = times::clock;

  explicit Counter(std::size_t fps_frame_count = 10)
    : count_(0), fps_(0), fps_frame_count_(fps_frame_count),
      color_count_(0), depth_count_(0), accel_count_(0), gyro_count_(0),
      gps_count_(0), ultrasonic_count_(0) {
    Init();
  }

  void Init() {
    time_beg_ = times::now();
    while (!fps_frame_times_.empty()) {
      fps_frame_times_.pop();
    }
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

  void IncrColorCount(const std::size_t& count = 1) {
    color_count_ += count;
  }

  void IncrDepthCount(const std::size_t& count = 1) {
    depth_count_ += count;
  }

  void IncrAccelCount(const std::size_t& count = 1) {
    accel_count_ += count;
  }

  void IncrGyroCount(const std::size_t& count = 1) {
    gyro_count_ += count;
  }

  void IncrGPSCount(const std::size_t& count = 1) {
    gps_count_ += count;
  }

  void IncrUltrasonicCount(const std::size_t& count = 1) {
    ultrasonic_count_ += count;
  }

  void PrintCountInfo() {
    auto &&time_end = times::now();
    float elapsed_ms =
        times::count<times::microseconds>(time_end - time_beg_) *
        0.001f;
    std::cout << "Time beg: " << times::to_local_string(time_beg_)
      << ", end: " << times::to_local_string(time_end)
      << ", cost: " << MillisReadable(elapsed_ms) << std::endl;

    if (color_count_ > 0) {
      std::cout << "Color count: " << color_count_
        << ", fps: " << (1000.f * color_count_ / elapsed_ms) << std::endl;
    }
    if (depth_count_ > 0) {
      std::cout << "Depth count: " << depth_count_
        << ", fps: " << (1000.f * depth_count_ / elapsed_ms) << std::endl;
    }
    if (accel_count_ > 0) {
      std::cout << "Accel count: " << accel_count_
        << ", hz: " << (1000.f * accel_count_ / elapsed_ms) << std::endl;
    }
    if (gyro_count_ > 0) {
      std::cout << "Gryo count: " << gyro_count_
        << ", hz: " << (1000.f * gyro_count_ / elapsed_ms) << std::endl;
    }
    if (gps_count_ > 0) {
      std::cout << "GPS count: " << gps_count_
        << ", hz: " << (1000.f * gps_count_ / elapsed_ms) << std::endl;
    }
    if (ultrasonic_count_ > 0) {
      std::cout << "Ultrasonic count: " << ultrasonic_count_
        << ", hz: " << (1000.f * ultrasonic_count_ / elapsed_ms) << std::endl;
    }
  }

  int64_t ElapsedMillis() const {
    return times::count<times::milliseconds>(times::now() - time_beg_);
  }

 private:
  std::string MillisReadable(float ms) {
    int ms_i = static_cast<int>(ms);
    float ms_l = ms - ms_i;
    std::ostringstream os;
    if (ms_i >= 86400000) {
      os << (ms_i / 86400000) << "d";
      ms_i = ms_i % 86400000;
    }
    if (ms_i >= 3600000) {
      os << (ms_i / 3600000) << "h";
      ms_i = ms_i % 3600000;
    }
    if (ms_i >= 60000) {
      os << (ms_i / 60000) << "m";
      ms_i = ms_i % 60000;
    }
    if (ms_i >= 1000) {
      os << (ms_i / 1000) << "s";
      ms_i = ms_i % 1000;
    }
    ms_l = ms_i + ms_l;
    if (ms_l > 0) {
      if (ms >= 1000) os << " ";
      os << ms_l << "ms";
    }
    return os.str();
  }

  clock::time_point time_beg_;
  std::uint64_t count_;
  double fps_;

  std::queue<clock::time_point> fps_frame_times_;
  std::size_t fps_frame_count_;

  std::size_t color_count_;
  std::size_t depth_count_;
  std::size_t accel_count_;
  std::size_t gyro_count_;
  std::size_t gps_count_;
  std::size_t ultrasonic_count_;

  // std::mutex mtx_;
};

}  // namespace util

MYNTEYE_END_NAMESPACE

// _kbhit()

#ifdef MYNTEYE_OS_WIN

#include <conio.h>

#else

#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
// #include <stropts.h>

int _kbhit() {
  static const int STDIN = 0;
  static bool initialized = false;

  if (!initialized) {
    // Use termios to turn off line buffering
    termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN, TCSANOW, &term);
    setbuf(stdin, NULL);
    initialized = true;
  }

  int bytes_waiting;
  ioctl(STDIN, FIONREAD, &bytes_waiting);
  return bytes_waiting;
}

#endif

#endif  // MYNTEYE_SAMPLES_UTIL_COUNTER_H_
