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
#include "mynteyed/utils.h"

#include <iomanip>
#include <iostream>
#include <vector>

MYNTEYE_BEGIN_NAMESPACE

namespace util {

bool select(const Camera& cam, DeviceInfo* info) {
  std::cout << std::endl;
  std::string dashes(80, '-');

  std::vector<DeviceInfo> dev_infos = cam.GetDeviceInfos();
  size_t n = dev_infos.size();
  if (n <= 0) {
    std::cerr << "Error: Device not found" << std::endl;
    return false;
  }

  std::cout << dashes << std::endl;
  std::cout << "Index | Device Information" << std::endl;
  std::cout << dashes << std::endl;
  for (auto &&info : dev_infos) {
    std::cout << std::setw(5) << info.index << " | " << info << std::endl;
  }
  std::cout << dashes << std::endl;

  if (n == 1) {
    *info = dev_infos[0];
    std::cout << "Auto select a device to open, index: 0" << std::endl;
  } else {
    size_t i;
    std::cout << "Please select a device to open, index: ";
    std::cin >> i;
    std::cout << std::endl;
    if (i < 0 || i >= n) {
      std::cerr << "Error: Index out of range" << std::endl;
      return false;
    }
    *info = dev_infos[i];
  }

  return true;
}

void print_stream_infos(const Camera& cam, const std::int32_t& dev_index) {
  std::string dashes(80, '-');

  std::vector<StreamInfo> color_infos;
  std::vector<StreamInfo> depth_infos;
  cam.GetStreamInfos(dev_index, &color_infos, &depth_infos);

  std::cout << dashes << std::endl;
  std::cout << "Index | Color Stream Information" << std::endl;
  std::cout << dashes << std::endl;
  for (auto &&info : color_infos) {
    std::cout << std::setw(5) << info.index << " | " << info << std::endl;
  }

  std::cout << dashes << std::endl;
  std::cout << "Index | Depth Stream Information" << std::endl;
  std::cout << dashes << std::endl;
  for (auto &&info : depth_infos) {
    std::cout << std::setw(5) << info.index << " | " << info << std::endl;
  }
  std::cout << dashes << std::endl;
}

bool is_right_color_supported(const StreamMode& mode) {
  return mode == StreamMode::STREAM_1280x480
      || mode == StreamMode::STREAM_2560x720;
}

}  // namespace util

MYNTEYE_END_NAMESPACE
