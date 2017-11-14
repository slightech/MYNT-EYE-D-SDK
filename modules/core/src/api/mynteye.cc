#include "mynteye.h"

using namespace mynteye;

std::ostream &operator<<(std::ostream &os, const StreamFormat &code) {
    switch (code) {
        case StreamFormat::STREAM_MJPG:
            os << "STREAM_MJPG"; break;
        case StreamFormat::STREAM_YUYV:
            os << "STREAM_YUYV"; break;
        case StreamFormat::STREAM_LAST:
            os << "STREAM_LAST"; break;
        default:
            os << "UNKNOWN"; break;
    }
    return os;
}
