#include <iostream>
#include <functional>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char const* argv[]) {
    Camera cam;
    DeviceInfo dev_info;
    if (!util::select(cam, &dev_info))
    {
        return 1;
    }
    util::print_stream_infos(cam, dev_info.index);

    std::cout << "Open device: index = " 
            << dev_info.index << ", name ="
            << dev_info.name <<  ", type = "
            << dev_info.type << ", pid = "
            << dev_info.pid << ", vid = "
            << dev_info.vid << ", fw_version = "
            << dev_info.fw_version << ", sn = "
            << dev_info.sn
            << std::endl;

    return 0;
}