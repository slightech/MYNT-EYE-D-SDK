#ifndef MYNTEYE_INTERNAL_SYNTHETIC_H_  // NOLINT
#define MYNTEYE_INTERNAL_SYNTHETIC_H_

#include <vector>
#include <memory>

#include "mynteye/callbacks.h"

MYNTEYE_BEGIN_NAMESPACE

class Channels;

class Synthetic {
 public:
  using stream_data_t = device::StreamData;
  using stream_datas_t = std::vector<stream_data_t>;

  void CaptureImage()
};

MYNTEYE_END_NAMESPACE

#endif // MYNTEYE_INTERNAL_SYNTHETIC_H_ // NOLINT
