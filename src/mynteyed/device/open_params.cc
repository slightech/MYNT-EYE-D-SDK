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
#include "mynteyed/device/open_params.h"

#include <utility>

#include "mynteyed/util/log.h"

MYNTEYE_USE_NAMESPACE

OpenParams::OpenParams() : OpenParams(0) {
}

OpenParams::OpenParams(const std::int32_t& dev_index)
  : dev_index(std::move(dev_index)),
    framerate(10),
    dev_mode(DeviceMode::DEVICE_ALL),
    color_mode(ColorMode::COLOR_RAW),
    depth_mode(DepthMode::DEPTH_COLORFUL),
    stream_mode(StreamMode::STREAM_1280x720),
    color_stream_format(StreamFormat::STREAM_YUYV),
    depth_stream_format(StreamFormat::STREAM_YUYV),
    state_ae(true),
    state_awb(true),
    ir_intensity(0),
    ir_depth_only(false),
    colour_depth_value(1000) {
  DBG_LOGD(__func__);
}

OpenParams::~OpenParams() {
  DBG_LOGD(__func__);
}
