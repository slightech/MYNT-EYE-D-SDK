#ifndef MYNTEYE_PACKAGE_H_
#define MYNTEYE_PACKAGE_H_
#pragma once

#include <algorithm>
#include "types.h"

MYNTEYE_BEGIN_NAMESPACE

namespace device {

class Image;

/**
 * @ingroup datatypes
 * Device motion data.
 */
struct MYNTEYE_API MotionData {
  /** ImuData. */
  std::shared_ptr<ImuData> imu;
};

/**
 * @ingroup datatypes
 * Image information data.
 */
struct MYNTEYE_API ImgInfoData {
  /** Image information */
  std::shared_ptr<ImgInfo> img_info;
}

/**
 * @ingroup datatypes
 * Device stream data.
 */
struct MYNTEYE_API StreamData {
  /** Image information */
  std::shared_ptr<ImgInfo> img_info;

  /** Image data */
  std::shared_ptr<Image> img;
};

}

MYNTEYE_END_NAMESPACE

#endif // MYNTEYE_PACKAGE_H_
