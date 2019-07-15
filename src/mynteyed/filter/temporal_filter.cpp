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
#include <mutex>
#include <array>
#include "mynteyed/filter/temporal_filter.h"

MYNTEYE_USE_NAMESPACE

const size_t PERSISTENCE_MAP_NUM = 9;

// The persistence parameter/holes filling mode
const uint8_t persistence_min = 0;
const uint8_t persistence_max = PERSISTENCE_MAP_NUM-1;
const uint8_t persistence_default = 7;
// Credible if two of the last four frames are valid at this pixel
const uint8_t persistence_step = 1;

// alpha -  the weight with default value 0.4,
// between 1 and 0 -- 1 means 100% weight from the current pixel
const float temp_alpha_min = 0.f;
const float temp_alpha_max = 1.f;
const float temp_alpha_default = 0.4f;
const float temp_alpha_step = 0.01f;

// delta -  the filter threshold for edge classification and
// preserving with default value of 20 depth increments
const uint16_t temp_delta_min = 10;
const uint16_t temp_delta_max = 2000;
const uint16_t temp_delta_default = 100;
const uint16_t temp_delta_step = 1;

bool TemporalFilter::process_frame(void* source) {
  temp_jw_smooth<uint16_t>(source, _last_frame.data(), _history.data());
  return true;
}

TemporalFilter::TemporalFilter() :
    _persistence_param(persistence_default),
    _alpha_param(temp_alpha_default),
    _one_minus_alpha(1- _alpha_param),
    _delta_param(temp_delta_default),
    _width(0), _height(0), _stride(0), _bpp(2),
    _current_frm_size_pixels(0) {
    TurnOn();
    on_set_persistence_control(_persistence_param);
    on_set_delta(_delta_param);
    on_set_alpha(_alpha_param);
}

void TemporalFilter::on_set_persistence_control(uint8_t val) {
  std::lock_guard<std::mutex> lock(_mutex);
  _persistence_param = val;
  recalc_persistence_map();
  _last_frame.clear();
  _history.clear();
}

void TemporalFilter::on_set_alpha(float val) {
  std::lock_guard<std::mutex> lock(_mutex);
  _alpha_param = val;
  _one_minus_alpha = 1.f - _alpha_param;
  _cur_frame_index = 0;
  _last_frame.clear();
  _history.clear();
}

void TemporalFilter::on_set_delta(float val) {
    std::lock_guard<std::mutex> lock(_mutex);
    _delta_param = static_cast<uint8_t>(val);
    _cur_frame_index = 0;
    _last_frame.clear();
    _history.clear();
}

void TemporalFilter::recalc_persistence_map() {
  _persistence_map.fill(0);
  for (size_t i = 0; i < _persistence_map.size(); i++) {
    unsigned char last_7 = !!(i & 1);  // old
    unsigned char last_6 = !!(i & 2);
    unsigned char last_5 = !!(i & 4);
    unsigned char last_4 = !!(i & 8);
    unsigned char last_3 = !!(i & 16);
    unsigned char last_2 = !!(i & 32);
    unsigned char last_1 = !!(i & 64);
    unsigned char lastFrame = !!(i & 128);  // new

    if (_persistence_param == 1) {
      int sum = lastFrame + last_1 + last_2 + last_3 +
          last_4 + last_5 + last_6 + last_7;
      if (sum >= 8)  // valid in eight of the last eight frames
        _persistence_map[i] = (uint8_t)1;
    } else if (_persistence_param == 2) {
      int sum = lastFrame + last_1 + last_2;
      if (sum >= 2)  // valid in two of the last three frames
          _persistence_map[i] = (uint8_t)1;
    } else if (_persistence_param == 3) {
      int sum = lastFrame + last_1 + last_2 + last_3;
      if (sum >= 2)  // valid in two of the last four frames
          _persistence_map[i] = (uint8_t)1;
    } else if (_persistence_param == 4) {
      int sum = lastFrame + last_1 + last_2 + last_3 +
          last_4 + last_5 + last_6 + last_7;
      if (sum >= 2)  // valid in two of the last eight frames
          _persistence_map[i] = (uint8_t)1;
    } else if (_persistence_param == 5) {
      int sum = lastFrame + last_1;
      if (sum >= 1)  // valid in one of the last two frames
          _persistence_map[i] = (uint8_t)1;
    } else if (_persistence_param == 6) {
      int sum = lastFrame + last_1 + last_2 + last_3 + last_4;
      if (sum >= 1)  // valid in one of the last five frames
          _persistence_map[i] = (uint8_t)1;
    } else if (_persistence_param == 7) {
      int sum = lastFrame + last_1 + last_2 + last_3 +
          last_4 + last_5 + last_6 + last_7;
      if (sum >= 1)  // valid in one of the last eight frames
          _persistence_map[i] = (uint8_t)1;
    } else if (_persistence_param == 8) {  //  <--- all 1's
        _persistence_map[i] = (uint8_t)1;
    } else {  // all others, including 0, no persistance
    }
  }

  // Convert to credible enough
  std::array<uint8_t, PRESISTENCY_LUT_SIZE> credible_threshold;
  credible_threshold.fill(0);

  for (auto phase = 0; phase < 8; phase++) {
    // evaluating last phase
    // int ephase = (phase + 7) % 8;
    unsigned char mask = 1 << phase;
    int i;

    for (i = 0; i < 256; i++) {
      unsigned char pos = (unsigned char)((i << (8 - phase)) | (i >> phase));
      if (_persistence_map[pos])
          credible_threshold[i] |= (uint8_t)mask;
    }
  }
  // Store results
  _persistence_map = credible_threshold;
}

bool TemporalFilter::LoadConfig(void* data) {
  uint8_t* persistence_control_p = reinterpret_cast<uint8_t*>(data);
  float* alpha_p = reinterpret_cast<float*>(persistence_control_p + 1);
  uint16_t* delta_p = reinterpret_cast<uint16_t*>(persistence_control_p + 5);
  on_set_persistence_control(*persistence_control_p);
  on_set_delta(*delta_p);
  on_set_alpha(*alpha_p);
  return true;
}

bool TemporalFilter::ProcessFrame(
    std::shared_ptr<Image> out,
    const std::shared_ptr<Image> in) {
  if (IsEnable()) {
    UpdateConfig(in->get_image_profile());
    if (out == in) {
      process_frame(in->data());
    } else {
      if (in->get_image_profile() == out->get_image_profile()) {
        auto tmp = in->Clone();
        process_frame(tmp->data());
        uint8_t *out_ptr = out->data();
        uint8_t *in_ptr = tmp->data();
        for (size_t i = 0; i < tmp->valid_size(); i++) {
          *out_ptr++ = *in_ptr++;
        }
      } else {
        out = in;
      }
      return true;
    }
    return true;
  } else {
    out = in;
    return false;
  }

  Enable(false);
  return false;
}

void TemporalFilter::UpdateConfig(
    const ImageProfile &in) {
  if (last_frame_profile == in) {
    return;
  }
  _bpp = in.bpp;
  _width = in.width;
  _height = in.height;
  _stride = _width*_bpp;
  _current_frm_size_pixels = _width * _height;

  _last_frame.clear();
  _last_frame.resize(_current_frm_size_pixels*_bpp);

  _history.clear();
  _history.resize(_current_frm_size_pixels*_bpp);

  last_frame_profile = in;
}

