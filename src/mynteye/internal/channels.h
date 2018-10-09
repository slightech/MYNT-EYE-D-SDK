#ifndef MYNTEYE_INTERNAL_CHANNELS_H_ // NOLINT
#define MYNTEYE_INTERNAL_CHANNELS_H_
#pragma once

#include <map>
#include <memory>
#include <thread>

#include "mynteye/internal/types.h"
#include "mynteye/internal/hid.h"

MYNTEYE_BEGIN_NAMESPACE

class hid_device;

class MYNTEYE_API Channels {
public:
  Channels();
  virtual ~Channels();

  using imu_callback_t = std::function<void(const ImuPacket &packet)>;
  using img_callback_t = std::function<void(const ImgInfoPacket &packet)>;

  void SetImuCallback(imu_callback_t callback);
  void SetImgInfoCallback(img_callback_t callback);
  void StartHidTracking();
  bool StopHidTracking();
  void DoHidTrack();

protected:
  bool ExtractHidData(ImuResPacket &imu, ImgInfoResPacket &img);
  int ReadHidData(std::uint8_t *data, int length);

private:
  std::shared_ptr<hid::hid_device> device_;

  bool is_hid_tracking_;
  volatile bool hid_track_stop_;
  std::thread hid_track_thread_;

  imu_callback_t imu_callback_;
  img_callback_t img_callback_;
};

MYNTEYE_END_NAMESPACE

#endif // MYNTEYE_INTERNAL_CHANNELS_H_
