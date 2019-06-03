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
#include "mynteyed/filter/spatial_filter.h"

MYNTEYE_USE_NAMESPACE

enum spatial_holes_filling_types : uint8_t
{
    sp_hf_disabled = 0,
    sp_hf_2_pixel_radius = 1,
    sp_hf_4_pixel_radius = 2,
    sp_hf_8_pixel_radius = 3,
    sp_hf_16_pixel_radius = 4,
    sp_hf_unlimited_radius = 5,
    sp_hf_max_value = 6
};

// The weight of the current pixel for smoothing is bounded within [25..100]%
const float alpha_min_val = 0.25f;
const float alpha_max_val = 1.f;
const float alpha_default_val = 0.6f;
const float alpha_step = 0.01f;

// The depth gradient below which the
// smoothing will occur as number of depth levels
const uint8_t delta_min_val = 1;
const uint8_t delta_max_val = 255;
const uint8_t delta_default_val = 100;
const uint8_t delta_step = 1;

// the number of passes used in the iterative smoothing approach
const uint8_t filter_iter_min = 1;
const uint8_t filter_iter_max = 5;
const uint8_t filter_iter_def = 3;
const uint8_t filter_iter_step = 1;

// The holes filling mode
const uint8_t holes_fill_min = sp_hf_disabled;
const uint8_t holes_fill_max = sp_hf_max_value - 1;
const uint8_t holes_fill_step = 1;
const uint8_t holes_fill_def = sp_hf_4_pixel_radius;

SpatialFilter::SpatialFilter() :
    _spatial_alpha_param(alpha_default_val),
    _spatial_delta_param(delta_default_val),
    _spatial_iterations(filter_iter_def),
    _width(0), _height(0), _stride(0), _bpp(2),
    _current_frm_size_pixels(0),
    _stereoscopic_depth(false),
    _focal_lenght_mm(0.f),
    _stereo_baseline_mm(0.f),
    _holes_filling_mode(holes_fill_def),
    _holes_filling_radius(0) {
  TurnOn();
  _spatial_edge_threshold = float(_spatial_delta_param);  // NOLINT
  switch (_holes_filling_mode) {
  case sp_hf_disabled:
    _holes_filling_radius = 0;      // disabled
    break;
  case sp_hf_unlimited_radius:
      // Unrealistic smearing; not particulary useful
    _holes_filling_radius = 0xff;
    break;
  case sp_hf_2_pixel_radius:
  case sp_hf_4_pixel_radius:
  case sp_hf_8_pixel_radius:
  case sp_hf_16_pixel_radius:
      // 2's exponential radius
    _holes_filling_radius = 0x1 << _holes_filling_mode;
    break;
  default:
    break;
  }
}
bool SpatialFilter::LoadConfig(void* data) {
  float* alpha_default_val_ptr = reinterpret_cast<float*>(data);
  uint8_t* mem_ptr = reinterpret_cast<uint8_t*>(data);
  uint8_t* delta_default_val_ptr = mem_ptr + 4;
  uint8_t* filter_iter_def_ptr = mem_ptr + 5;
  uint8_t* holes_fill_def_ptr = mem_ptr + 6;
  _spatial_alpha_param = *alpha_default_val_ptr;
  _spatial_delta_param = *delta_default_val_ptr;
  _spatial_iterations = *filter_iter_def_ptr;
  _holes_filling_mode = *holes_fill_def_ptr;
  _spatial_edge_threshold = float(_spatial_delta_param);  // NOLINT
  switch (_holes_filling_mode) {
  case sp_hf_disabled:
    _holes_filling_radius = 0;      // disabled
    break;
  case sp_hf_unlimited_radius:
      // Unrealistic smearing; not particulary useful
    _holes_filling_radius = 0xff;
    break;
  case sp_hf_2_pixel_radius:
  case sp_hf_4_pixel_radius:
  case sp_hf_8_pixel_radius:
  case sp_hf_16_pixel_radius:
      // 2's exponential radius
    _holes_filling_radius = 0x1 << _holes_filling_mode;
    break;
  default:
    break;
  }
  return true;
}

void SpatialFilter::process_frame(void* source) {
  dxf_smooth<uint16_t>(
      source, _spatial_alpha_param,
      _spatial_edge_threshold, _spatial_iterations);
}

void SpatialFilter::recursive_filter_horizontal_fp(
  void * image_data, float alpha, float deltaZ) {
  float *image = reinterpret_cast<float*>(image_data);

  unsigned int v, u;

  for (v = 0; v < _height;) {
    // left to right
    float *im = image + v * _width;
    float state = *im;
    union {
      float previousInnovation;
      int tmp_flag_previousInnovation;
    };
    previousInnovation = state;

    im++;
    union {
      float innovation;
      int tmp_flag_innovation;
    };
    innovation = *im;
    u = int(_width) - 1;  // NOLINT
    if (!(tmp_flag_previousInnovation > 0))  // NOLINT
        goto CurrentlyInvalidLR;
    // else fall through

  CurrentlyValidLR:
    for (;;) {
      if (tmp_flag_innovation > 0) {
        float delta = previousInnovation - innovation;
        bool smallDifference = delta < deltaZ && delta > -deltaZ;

        if (smallDifference) {
          float filtered = innovation * alpha + state * (1.0f - alpha);
          *im = state = filtered;
        } else {
          state = innovation;
        }
        u--;
        if (u <= 0)
            goto DoneLR;
        previousInnovation = innovation;
        im += 1;
        innovation = *im;
      } else {  // switch to CurrentlyInvalid state
        u--;
        if (u <= 0)
            goto DoneLR;
        previousInnovation = innovation;
        im += 1;
        innovation = *im;
        goto CurrentlyInvalidLR;
      }
    }

CurrentlyInvalidLR:
    for (;;) {
      u--;
      if (u <= 0)
        goto DoneLR;
      if (tmp_flag_innovation > 0) {  // switch to CurrentlyValid state
        previousInnovation = state = innovation;
        im += 1;
        innovation = *im;
        goto CurrentlyValidLR;
      } else {
        im += 1;
        innovation = *im;
      }
    }
DoneLR:

    // right to left
    im = image + (v + 1) * _width - 2;  // end of row - two pixels
    previousInnovation = state = im[1];
    u = int(_width) - 1;   // NOLINT
    innovation = *im;
    if (!(tmp_flag_previousInnovation > 0))
        goto CurrentlyInvalidRL;
        // else fall through
CurrentlyValidRL:
    for (;;) {
      if (tmp_flag_innovation > 0) {  // NOLINT
        float delta = previousInnovation - innovation;
        bool smallDifference = delta < deltaZ && delta > -deltaZ;

        if (smallDifference) {
            float filtered = innovation * alpha + state * (1.0f - alpha);
            *im = state = filtered;
        } else {
            state = innovation;
        }
        u--;
        if (u <= 0)
            goto DoneRL;
        previousInnovation = innovation;
        im -= 1;
        innovation = *im;
      } else {  // switch to CurrentlyInvalid state
        u--;
        if (u <= 0)
            goto DoneRL;
        previousInnovation = innovation;
        im -= 1;
        innovation = *im;
        goto CurrentlyInvalidRL;
      }
    }

CurrentlyInvalidRL:
    for (;;) {
      u--;
      if (u <= 0)
          goto DoneRL;
      if (tmp_flag_innovation > 0) {  // switch to CurrentlyValid state  // NOLINT
          previousInnovation = state = innovation;
          im -= 1;
          innovation = *im;
          goto CurrentlyValidRL;
      } else {
          im -= 1;
          innovation = *im;
      }
    }
DoneRL:
    v++;
  }
}

void SpatialFilter::recursive_filter_vertical_fp(
    void * image_data, float alpha, float deltaZ) {
  float *image = reinterpret_cast<float*>(image_data);

  unsigned int v, u;

  // we'll do one column at a time,
  // top to bottom, bottom to top, left to right,

  for (u = 0; u < _width;) {
    float *im = image + u;
    float state = im[0];
    union {
      float previousInnovation;
      int tmp_flag_previousInnovation;
    };
    previousInnovation = state;

    v = int(_height) - 1;  // NOLINT
    im += _width;
    union {
      float innovation;
      int tmp_flag_innovation;
    };
    innovation = *im;

    if (!(tmp_flag_previousInnovation > 0))  //NOLINT
        goto CurrentlyInvalidTB;
    // else fall through

CurrentlyValidTB:
    for (;;) {
      if (tmp_flag_innovation > 0) {
        float delta = previousInnovation - innovation;
        bool smallDifference = delta < deltaZ && delta > -deltaZ;

        if (smallDifference) {
            float filtered = innovation * alpha + state * (1.0f - alpha);
            *im = state = filtered;
        } else {
            state = innovation;
        }
        v--;
        if (v <= 0)
            goto DoneTB;
        previousInnovation = innovation;
        im += _width;
        innovation = *im;
      } else {  // switch to CurrentlyInvalid state
        v--;
        if (v <= 0)
            goto DoneTB;
        previousInnovation = innovation;
        im += _width;
        innovation = *im;
        goto CurrentlyInvalidTB;
      }
    }

CurrentlyInvalidTB:
    for (;;) {
      v--;
      if (v <= 0) goto DoneTB;
      if (tmp_flag_innovation > 0) {  // switch to CurrentlyValid state  // NOLINT
        previousInnovation = state = innovation;
        im += _width;
        innovation = *im;
        goto CurrentlyValidTB;
      } else {
        im += _width;
        innovation = *im;
      }
    }
DoneTB:

    im = image + u + (_height - 2) * _width;
    state = im[_width];
    previousInnovation = state;
    innovation = *im;
    v = int(_height) - 1;  // NOLINT
    if (!(tmp_flag_innovation > 0))  // NOLINT
        goto CurrentlyInvalidBT;
        // else fall through
CurrentlyValidBT:
    for (;;) {
      if (tmp_flag_innovation > 0) {  // NOLINT
        float delta = previousInnovation - innovation;
        bool smallDifference = delta < deltaZ && delta > -deltaZ;

        if (smallDifference) {
            float filtered = innovation * alpha + state * (1.0f - alpha);
            *im = state = filtered;
        } else {
            state = innovation;
        }
        v--;
        if (v <= 0)
            goto DoneBT;
        previousInnovation = innovation;
        im -= _width;
        innovation = *im;
      } else {  // switch to CurrentlyInvalid state
        v--;
        if (v <= 0)
            goto DoneBT;
        previousInnovation = innovation;
        im -= _width;
        innovation = *im;
        goto CurrentlyInvalidBT;
      }
    }

CurrentlyInvalidBT:
    for (;;) {
      v--;
      if (v <= 0)
          goto DoneBT;
      if (tmp_flag_innovation > 0) { // switch to CurrentlyValid state // NOLINT
          previousInnovation = state = innovation;
          im -= _width;
          innovation = *im;
          goto CurrentlyValidBT;
      } else {
        im -= _width;
        innovation = *im;
      }
    }
DoneBT:
    u++;
  }
}

bool SpatialFilter::ProcessFrame(
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
        return false;
      }
    }
    return true;
  } else {
    out = in;
    return false;
  }

  Enable(false);
  return false;
}

void SpatialFilter::UpdateConfig(
    const ImageProfile &in) {
  if (last_frame_profile == in) {
    return;
  }
  _bpp = in.bpp;
  _width = in.width;
  _height = in.height;
  _stride = _width*_bpp;
  _current_frm_size_pixels = _width * _height;
  _tmp_frame.clear();
  _tmp_frame.resize(_current_frm_size_pixels*_bpp);

  last_frame_profile = in;
}

