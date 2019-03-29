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
#include <iostream>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"
#include "mynteyed/util/rate.h"

#include "util/pc_utils.h"

#define CAMERA_FACTOR 1000.0

MYNTEYE_USE_NAMESPACE

int main(int argc, char const* argv[]) {
  Camera cam;
  DeviceInfo dev_info;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  util::print_stream_infos(cam, dev_info.index);

  std::cout << "Open device: " << dev_info.index << ", "
      << dev_info.name << std::endl << std::endl;

  OpenParams params(dev_info.index);
  params.color_mode = ColorMode::COLOR_RECTIFIED;
  // Note: must set DEPTH_RAW to get raw depth values for points
  params.depth_mode = DepthMode::DEPTH_RAW;
  params.stream_mode = StreamMode::STREAM_1280x720;
  params.ir_intensity = 4;

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl;

  Rate rate(params.framerate);
  util::PCViewer viewer(1280, 720);
  for (;;) {
    auto cloud = util::get_point_cloud(&cam, CAMERA_FACTOR);
    if (cloud) viewer.Update(cloud);
    if (viewer.WasStopped()) break;
    rate.Sleep();
  }

  cam.Close();
  return 0;
}
