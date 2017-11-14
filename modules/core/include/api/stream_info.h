#ifndef MYNTEYE_API_STREAM_INFO_H_
#define MYNTEYE_API_STREAM_INFO_H_
#pragma once

#include <string>

#include "mynteye.h"

namespace mynteye {

/**
 * Stream information.
 */
struct MYNTEYE_API StreamInfo {

    /**
     * The stream index.
     */
    std::int32_t index;

    /**
     * The stream width.
     */
    std::int32_t width;

    /**
     * The stream height.
     */
    std::int32_t height;

    /**
     * The stream format.
     */
    StreamFormat format;

};

}  // namespace mynteye

MYNTEYE_API std::ostream &operator<<(std::ostream &os, const mynteye::StreamInfo &info);

#endif  // MYNTEYE_API_STREAM_INFO_H_
