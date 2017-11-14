#include "init_params.h"

#include "log.hpp"

using namespace mynteye;

InitParams::InitParams(const DeviceInfo &info)
    : dev_info(std::move(info)),
      framerate(30),
      depth_mode(DepthMode::DEPTH_COLORFUL),
      color_info_index(-1),
      depth_info_index(-1),
      state_ae(true),
      state_awb(true) {
    DBG_LOGD(__func__);
}

InitParams::~InitParams() {
    DBG_LOGD(__func__);
}
