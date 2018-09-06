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
#ifndef MYNTEYE_API_DEV_INFO_H_
#define MYNTEYE_API_DEV_INFO_H_
#pragma once

#include <string>

#include "mynteye/mynteye.h"

namespace mynteye {

/**
 * Device information.
 */
struct MYNTEYE_API DeviceInfo {

    /**
     * The device index.
     */
    std::int32_t index;

    /**
     * The device name.
     */
    std::string name;

    /**
     * The device type.
     */
    std::uint16_t type;

    /**
     * The product id.
     */
    std::uint16_t pid;

    /**
     * The vendor id.
     */
    std::uint16_t vid;

    /**
     * The chip id.
     */
    std::uint16_t chip_id;

    /**
     * The firmware version.
     */
    std::string fw_version;

};

}  // namespace mynteye

MYNTEYE_API std::ostream &operator<<(std::ostream &os, const mynteye::DeviceInfo &info);

#endif  // MYNTEYE_API_DEV_INFO_H_
