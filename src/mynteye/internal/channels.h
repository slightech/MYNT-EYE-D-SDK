#ifndef MYNTEYE_INTERNAL_CHANNELS_H_ // NOLINT
#define MYNTEYE_INTERNAL_CHANNELS_H_
#pragma once

#include <map>
#include <memory>
#include <thread>

#include "mynteye/types.h"
#include "mynteye/internal/types.h"
#include "mynteye/internal/hid.h"

MYNTEYE_BEGIN_NAMESPACE

class hid_device;

class MYNTEYE_API Channels {
 public:
  typedef enum FileId {
    FID_DEVICE_INFO = 1,  // device info
    FID_RESERVE = 2,   // reserve
    FID_IMU_PARAMS = 4,   // imu intrinsics & extrinsics
    FID_LAST,
  } file_id_t;

  using device_info_t = DeviceParams;

  typedef struct ImuParams {
    bool ok;
    ImuIntrinsics in_accel;
    ImuIntrinsics in_gyro;
    Extrinsics ex_left_to_imu;
  } imu_params_t;

  Channels();
  virtual ~Channels();

  using imu_callback_t = std::function<void(const ImuPacket &packet)>;
  using img_callback_t = std::function<void(const ImgInfoPacket &packet)>;

  void SetImuCallback(imu_callback_t callback);
  void SetImgInfoCallback(img_callback_t callback);
  void Open();
  void StartHidTracking();
  bool StopHidTracking();
  void DoHidTrack();

  bool GetFiles(device_info_t *info,
    imu_params_t *imu_params,
    Version *spec_version = nullptr);

  bool SetFiles(device_info_t *info,
      imu_params_t *imu_params,
      Version *spec_version);

 protected:
  bool ExtractHidData(ImuResPacket &imu, ImgInfoResPacket &img);
  bool RequireFileData(bool device_info,
      bool reserve,
      bool imu_params,
      std::uint8_t *data,
      std::uint16_t &file_size);
  bool UpdateFileData(std::uint8_t *data, std::uint16_t size);

 private:
  std::shared_ptr<hid::hid_device> device_;

  bool is_hid_tracking_;
  volatile bool hid_track_stop_;
  std::thread hid_track_thread_;

  imu_callback_t imu_callback_;
  img_callback_t img_callback_;

  std::uint8_t req_count_ = 0;
};

MYNTEYE_END_NAMESPACE

#endif // MYNTEYE_INTERNAL_CHANNELS_H_
