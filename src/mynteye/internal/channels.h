#ifndef MYNTEYE_INTERNAL_CHANNELS_H_ // NOLINT
#define MYNTEYE_INTERNAL_CHANNELS_H_
#pragma once

#include <map>
#include <memory>
#include <thread>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

#include "internal/types.h"
#include "hid.h"

MYNTEYE_BEGIN_NAMESPACE

class MYNTEYE_API Channels {
public:
  Channels();
  virtual ~Channels();

  using imu_callback_t = std::function<void(const ImuPacket &packet)>;
  using img_callback_t = std::function<void(const ImgInfoPacket &packet)>;

  void SetImuCallback(imu_callback_t callback);
  void SetImgInfoCallback(img_callback_t callback);
  bool StartHidTracking();
  bool StopHidTracking();

protected:
  void DoHidTrack();
  void ExtractHidData(ImuResPacket &res);
  void ReadHidData(std::uint8_t *data, int length);

private:
  std::shared_ptr<hid::hid_device> device_;

  bool is_hid_tracking_;
  volatile bool hid_track_stop_;
  std::thread hid_track_thread_;

  imu_callback_t imu_callback_;
  img_callback_t img_callback_;
};
