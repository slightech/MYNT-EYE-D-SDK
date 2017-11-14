#ifndef MYNTEYE_API_INIT_PARAMS_H_
#define MYNTEYE_API_INIT_PARAMS_H_
#pragma once

#include <string>

#include "dev_info.h"
#include "mynteye.h"
#include "stream_info.h"

namespace mynteye {

/**
 * Initialization Parameters.
 */
struct MYNTEYE_API InitParams {

    /**
     * Device info.
     */
    DeviceInfo dev_info;

    /**
     * Framerate, default 30.
     */
    std::int32_t framerate;

    /**
     * Depth mode, default ::DEPTH_COLORFUL
     */
    DepthMode depth_mode;

    /**
     * Color stream info, default -1
     */
    std::int32_t color_info_index;

    /**
     * Depth stream info, default -1
     */
    std::int32_t depth_info_index;

    /**
     * Auto-exposure, default true.
     */
    bool state_ae;

    /**
     * Auto-white balance, default true.
     */
    bool state_awb;

    /** Constructor. */
    InitParams(const DeviceInfo &info);

    /** Destructor. */
    ~InitParams();

};

}  // namespace mynteye

#endif  // MYNTEYE_API_INIT_PARAMS_H_
