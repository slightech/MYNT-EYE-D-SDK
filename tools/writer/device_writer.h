#ifndef TOOLS_DEVICE_WRITER_H_ // NOLINT
#define TOOLS_DEVICE_WRITER_H_
#pragma once

#include <memory>
#include <string>

#include "mynteye/stubs/global.h"
#include "mynteye/internal/channels.h"
#include "mynteye/internal/camera_p.h"
#include "mynteye/internal/types.h"

MYNTEYE_BEGIN_NAMESPACE

class CameraPrivate;

namespace tools {

class DeviceWriter {
 public:
  using dev_info_t = DeviceParams;
  using imu_params_t = Channels::imu_params_t;

  explicit DeviceWriter(std::shared_ptr<CameraPrivate> device);
  ~DeviceWriter();

  bool WriteDeviceInfo(const dev_info_t &info);
  bool WriteDeviceInfo(const std::string &filepath);

  bool WriteImuParams(const imu_params_t &params);
  bool WriteImuParams(const std::string &filepath);

  bool SaveDeviceInfo(const dev_info_t &info, const std::string &filepath);
  bool SaveImuParams(const imu_params_t &params, const std::string &filepath);

  /** Save all infos of this device */
  void SaveAllInfos(const std::string &dir);

 private:
  dev_info_t LoadDeviceInfo(const std::string &filepath);
  imu_params_t LoadImuParams(const std::string &filepath);

  std::shared_ptr<CameraPrivate> device_;
};

} // namespace tools

MYNTEYE_END_NAMESPACE

#endif // TOOLS_DEVICE_WRITER_H_ // NOLINT
