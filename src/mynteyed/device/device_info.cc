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
#include "mynteyed/device/device_info.h"

#include <iomanip>
#include <iostream>

MYNTEYE_BEGIN_NAMESPACE

std::ostream& operator<<(std::ostream& os, const DeviceInfo& info) {
  std::ios fmt{nullptr};
  fmt.copyfmt(os);  // clear format
  os << "index: " << info.index
    << ", name: " << info.name
    << ", type: " << info.type
    << ", pid: 0x" << std::hex << info.pid
    << ", vid: 0x" << std::hex << info.vid
    << ", chip_id: 0x" << std::hex << info.chip_id
    << ", fw_version: " << info.fw_version
    << ", serial_number: " << info.sn;
  os.copyfmt(fmt);  // restore format
  return os;
}

MYNTEYE_END_NAMESPACE
