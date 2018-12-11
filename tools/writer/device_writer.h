#ifndef TOOLS_DEVICE_WRITER_H_ // NOLINT
#define TOOLS_DEVICE_WRITER_H_
#pragma once

#include <memory>
#include <string>

#include "mynteyed/camera.h"

MYNTEYE_BEGIN_NAMESPACE

class CameraPrivate;

namespace tools {

class DeviceWriter {
 public:
  using device_desc_t = device::Descriptors;
  using imu_params_t = device::ImuParams;

  explicit DeviceWriter(std::shared_ptr<Camera> device);
  ~DeviceWriter();

  bool WriteDescriptors(const device_desc_t &desc);
  bool WriteDescriptors(const std::string &filepath);

  bool WriteImuParams(const imu_params_t &params);
  bool WriteImuParams(const std::string &filepath);

  bool SaveDescriptors(const device_desc_t &desc, const std::string &filepath);
  bool SaveImuParams(const imu_params_t &params, const std::string &filepath);

  /** Save all datas of this device */
  void SaveAllDatas(const std::string &dir);

  bool HidFirmwareUpdate(const char *filepath);

 private:
  device_desc_t LoadDescriptors(const std::string &filepath);
  imu_params_t LoadImuParams(const std::string &filepath);

  std::shared_ptr<Camera> device_;
};

}  // namespace tools

MYNTEYE_END_NAMESPACE

#endif // TOOLS_DEVICE_WRITER_H_ // NOLINT
