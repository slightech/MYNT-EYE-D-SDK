#include "stream_info.h"

using namespace mynteye;

std::ostream &operator<<(std::ostream &os, const StreamInfo &info) {
    os << "index: " << info.index
        << ", width: " << info.width
        << ", height: " << info.height
        << ", format: " << info.format;
    return os;
}
