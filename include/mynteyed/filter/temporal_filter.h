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

#pragma once
#include <stdint.h>
#include <mutex>
#include <limits>
#include <cmath>
#include <cassert>
#include <cstring>
#include <vector>
#include <sstream>
#include <memory>
#include <map>
#include <algorithm>
#include <condition_variable>
#include <functional>
#include <utility>
#include <iomanip>
#include "mynteyed/filter/base_filter.h"

MYNTEYE_BEGIN_NAMESPACE

const size_t PRESISTENCY_LUT_SIZE = 256;

class MYNTEYE_API TemporalFilter : public BaseFilter{
 public:
  TemporalFilter();
  bool LoadConfig(void* data) override;
  bool ProcessFrame(
      std::shared_ptr<Image> out,
      const std::shared_ptr<Image> in) override;

 protected:
  template<typename T>
  void temp_jw_smooth(
      void* frame_data, void * _last_frame_data, uint8_t *history) {
    static_assert(
        (std::is_arithmetic<T>::value),
        "temporal filter assumes numeric types");
    T delta_z = static_cast<T>(_delta_param);

    auto frame          = reinterpret_cast<T*>(frame_data);
    auto _last_frame    = reinterpret_cast<T*>(_last_frame_data);

    unsigned char mask = 1 << _cur_frame_index;

    // pass one -- go through image and update all
    for (size_t i = 0; i < _current_frm_size_pixels; i++) {
      T cur_val = frame[i];
      T prev_val = _last_frame[i];

      if (cur_val) {
        if (!prev_val) {
          _last_frame[i] = cur_val;
          history[i] = mask;
        } else {  // old and new val
        T diff = static_cast<T>(fabs(cur_val - prev_val));
          if (diff < delta_z) {  // old and new val agree
            history[i] |= mask;
            float filtered =
              _alpha_param * cur_val + _one_minus_alpha * prev_val;
            T result = static_cast<T>(filtered);
            frame[i] = result;
            _last_frame[i] = result;
          } else {
            _last_frame[i] = cur_val;
            history[i] = mask;
          }
        }
      } else {  // no cur_val
        if (prev_val) {  // only case we can help
          unsigned char hist = history[i];
          unsigned char classification = _persistence_map[hist];
          if (classification & mask) {  // we have had enough samples lately
            frame[i] = prev_val;
          }
        }
        history[i] &= ~mask;
      }
    }
    _cur_frame_index = (_cur_frame_index + 1) % 8;  // at end of cycle
}

  void on_set_persistence_control(uint8_t val);
  void on_set_alpha(float val);
  void on_set_delta(float val);

  void recalc_persistence_map();

 private:
  bool process_frame(void* source);
  void UpdateConfig(const ImageProfile &in);
  uint8_t _persistence_param;

  float _alpha_param;  // The normalized weight of the current pixel
  float _one_minus_alpha;
  uint16_t _delta_param;  // A threshold when a filter is invoked
  size_t _width, _height, _stride;
  size_t _bpp;
  size_t _current_frm_size_pixels;
  std::vector<uint8_t> _last_frame;
  // Hold the last frame received for the current profile
  std::vector<uint8_t>  _history;
  // represents the history over the last 8 frames, 1 bit per frame
  uint8_t _cur_frame_index;
  // encodes whether a particular 8 bit history
  // is good enough for all 8 phases of storage
  std::array<uint8_t, PRESISTENCY_LUT_SIZE> _persistence_map;
  std::mutex _mutex;
  ImageProfile            last_frame_profile;
};

MYNTEYE_END_NAMESPACE
