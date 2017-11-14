#ifndef MYNTEYE_API_DEV_INFO_H_
#define MYNTEYE_API_DEV_INFO_H_
#pragma once

#include <string>

#include "mynteye.h"

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
