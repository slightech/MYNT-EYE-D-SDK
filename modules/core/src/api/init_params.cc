#include "init_params.h"

#include "log.hpp"

using namespace mynteye;

InitParams::InitParams() {
}

InitParams::InitParams(const std::int32_t &dev_index)
    : dev_index(std::move(dev_index)),
      framerate(30),
      depth_mode(DepthMode::DEPTH_COLORFUL),
      color_info_index(-1),
      depth_info_index(-1),
      state_ae(true),
      state_awb(true),
      ir_intensity(0) {
    DBG_LOGD(__func__);
}

InitParams::~InitParams() {
    DBG_LOGD(__func__);
}
