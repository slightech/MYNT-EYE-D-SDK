#include "dev_info.h"

#include <iomanip>

using namespace mynteye;

std::ostream &operator<<(std::ostream &os, const DeviceInfo &info) {
    std::ios fmt{nullptr};
    fmt.copyfmt(os);
    os << "index: " << info.index
        << ", name: " << info.name
        << ", type: " << info.type
        << ", pid: 0x" << std::hex << info.pid
        << ", vid: 0x" << std::hex << info.vid
        << ", chip_id: 0x" << std::hex << info.chip_id
        << ", fw_version: " << info.fw_version;
    os.copyfmt(fmt);
    return os;
}
