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
#include "mynteyed/util/rate.h"

#include <thread>

MYNTEYE_USE_NAMESPACE

Rate::Rate(std::int32_t frequency)
  : time_beg_(times::now()),
    expected_cycle_time_(clock::period::den / clock::period::num / frequency),
    actual_cycle_time_(0) {
}

Rate::~Rate() {
}

void Rate::Sleep() {
  auto expected_end = time_beg_ + expected_cycle_time_;

  auto actual_end = times::now();

  // detect backward jumps in time
  if (actual_end < time_beg_) {
    expected_end = actual_end + expected_cycle_time_;
  }

  // calculate the time we'll sleep for
  auto sleep_time = expected_end - actual_end;

  // set the actual amount of time the loop took in case the user wants to know
  actual_cycle_time_ = actual_end - time_beg_;

  // make sure to reset our start time
  time_beg_ = expected_end;

  // if we've taken too much time we won't sleep
  if (sleep_time <= clock::duration(0)) {
    // if we've jumped forward in time, or the loop has taken more than a full
    // extra cycle, reset our cycle
    if (actual_end > expected_end + expected_cycle_time_) {
      time_beg_ = actual_end;
    }
    return;
  }

  std::this_thread::sleep_for(sleep_time);
}

void Rate::Reset() {
  time_beg_ = times::now();
}

Rate::clock::duration Rate::CycleTime() {
  return actual_cycle_time_;
}

Rate::clock::duration Rate::ExpectedCycleTime() {
  return expected_cycle_time_;
}
