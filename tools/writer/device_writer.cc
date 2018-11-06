#include <vector>

#include <opencv2/core/core.hpp>

#include "writer/device_writer.h"
#include "mynteye/util/files.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

namespace tools {

DeviceWriter::DeviceWriter(std::shared_ptr<CameraPrivate> device) : device_(device) {
}

DeviceWriter::~DeviceWriter() {
}

bool DeviceWriter::WriteDeviceInfo(const dev_info_t &info) {
  auto &&channels = device_->channels();
  channels->Open();
  auto &&dev_info = device_->GetInfo();
  if (nullptr == dev_info) {
    std::cerr << "Device was not initialization." << std::endl;
    return false;
  }
  dev_info->lens_type = Type(info.lens_type);
  dev_info->imu_type = Type(info.imu_type);
  dev_info->nominal_baseline = info.nominal_baseline;
  if (channels->SetFiles(dev_info.get(), nullptr, nullptr)) {
    std::cout << std::endl;
    std::cout << "Write device info success" << std::endl;
    std::cout << "Device info: {name: " << dev_info->name
      << " serial_number: " << dev_info->serial_number
      << " firmware_version: " << dev_info->firmware_version.to_string()
      << " hardware_version: " << dev_info->hardware_version.to_string()
      << " spec_version: " << dev_info->spec_version.to_string()
      << " lens_type: " << dev_info->lens_type.to_string()
      << " imu_type: " << dev_info->imu_type.to_string()
      << " nominal_baseline: " << dev_info->nominal_baseline
      << "}" << std::endl;
    std::cout << std::endl;
    return true;
  } else {
    std::cerr << "Write device info failed" << std::endl;
    return false;
  }
}

bool DeviceWriter::WriteDeviceInfo(const std::string &filepath) {
  return WriteDeviceInfo(LoadDeviceInfo(filepath));
}

bool DeviceWriter::WriteImuParams(const imu_params_t &params) {
  auto &&channels = device_->channels();
  channels->Open();
  auto &&dev_info = device_->GetInfo();
  if (nullptr == dev_info) {
    std::cerr << "Device was not initialization." << std::endl;
    return false;
  }
  if (channels->SetFiles(dev_info.get(),
        const_cast<imu_params_t *>(&params), &dev_info->spec_version)) {
    std::cout << std::endl;
    std::cout << "Write imu params success" << std::endl;
    std::cout << "Imu intrinsics accel: {" << params.in_accel << "}" << std::endl;
    std::cout << "Imu intrinsics gyro: {" << params.in_gyro << "}" << std::endl;
    std::cout << "Imu extrinsics left to imu: {" << params.ex_left_to_imu << "}" << std::endl;
    std::cout << std::endl;
    return true;
  } else {
    std::cerr << "Write imu params failed" << std::endl;
    return false;
  }
}

bool DeviceWriter::WriteImuParams(const std::string &filepath) {
  return WriteImuParams(LoadImuParams(filepath));
}

namespace {

cv::FileStorage &operator<<(cv::FileStorage &fs, const ImuIntrinsics &in) {
  std::vector<double> scales;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      scales.push_back(in.scale[i][j]);
    }
  }
  std::vector<double> assembly;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      assembly.push_back(in.assembly[i][j]);
    }
  }
  fs << "{"
     << "scale" << scales << "assembly" << assembly << "drift"
     << std::vector<double>(in.drift, in.drift + 3) << "noise"
     << std::vector<double>(in.noise, in.noise + 3) << "bias"
     << std::vector<double>(in.bias, in.bias + 3) << "x"
     << std::vector<double>(in.x, in.x + 2) << "y"
     << std::vector<double>(in.y, in.y + 2) << "z"
     << std::vector<double>(in.z, in.z + 2) << "}";
  return fs;
}

cv::FileStorage &operator<<(cv::FileStorage &fs, const Extrinsics &ex) {
  std::vector<double> rotations;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      rotations.push_back(ex.rotation[i][j]);
    }
  }
  fs << "{"
     << "rotation" << rotations << "translation"
     << std::vector<double>(ex.translation, ex.translation + 3) << "}";
  return fs;
}

} // namespace

bool DeviceWriter::SaveDeviceInfo(
    const dev_info_t &info, const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cout << "Failed to save file: " << filepath << std::endl;
    return false;
  }
  fs << "device_name" << info.name;
  fs << "serial_number" << info.serial_number;
  fs << "firmware_version" << info.firmware_version.to_string();
  fs << "hardware_version" << info.hardware_version.to_string();
  fs << "spec_version" << info.spec_version.to_string();
  fs << "lens_type" << info.lens_type.to_string();
  fs << "imu_type" << info.imu_type.to_string();
  fs << "nominal_baseline" << info.nominal_baseline;
  fs.release();
  return true;
}

bool DeviceWriter::SaveImuParams(
    const imu_params_t &params, const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cout << "Failed to save file: " << filepath << std::endl;
    return false;
  }
  fs << "in_accel" << params.in_accel << "in_gyro" << params.in_gyro
     << "ex_left_to_imu" << params.ex_left_to_imu;
  fs.release();
  return true;
}

void DeviceWriter::SaveAllInfos(const std::string &dir) {
  if (!files::mkdir(dir)) {
    std::cout << "Create directory failed: " << std::endl;
  }
  SaveDeviceInfo(*device_->GetInfo(), dir + MYNTEYE_OS_SEP "device.info");
  auto &&m_in = device_->GetMotionIntrinsics();
  SaveImuParams(
      {
          false, m_in.accel, m_in.gyro,
          device_->GetMotionExtrinsics(),
      },
      dir + MYNTEYE_OS_SEP "imu.params");
}

namespace {

void to_extrinsics(const cv::Mat &R, const cv::Mat &T, Extrinsics *ex) {
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      ex->rotation[i][j] = R.at<double>(i, j);
    }
  }
  for (std::size_t i = 0; i < 3; i++) {
    ex->translation[i] = T.at<double>(i);
  }
}

void operator>>(const cv::FileNode &n, ImuIntrinsics &in) {
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      in.scale[i][j] = n["scale"][3 * i + j];
    }
  }
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      in.assembly[i][j] = n["assembly"][3 * i + j];
    }
  }
  for (std::size_t i = 0; i < 3; i++) {
    in.drift[i] = n["drift"][i];
  }
  for (std::size_t i = 0; i < 3; i++) {
    in.noise[i] = n["noise"][i];
  }
  for (std::size_t i = 0; i < 3; i++) {
    in.bias[i] = n["bias"][i];
  }

  for (std::size_t i = 0; i < 2; i++) {
    in.x[i] = n["x"][i];
  }
  for (std::size_t i = 0; i < 2; i++) {
    in.y[i] = n["y"][i];
  }
  for (std::size_t i = 0; i < 2; i++) {
    in.z[i] = n["z"][i];
  }
}

void operator>>(const cv::FileNode &n, Extrinsics &ex) {
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      ex.rotation[i][j] = n["rotation"][3 * i + j];
    }
  }
  for (std::size_t i = 0; i < 3; i++) {
    ex.translation[i] = n["translation"][i];
  }
}

} // namespace

DeviceWriter::dev_info_t DeviceWriter::LoadDeviceInfo(
    const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Failed to load file: " << filepath << std::endl;
  }
  DeviceParams info;
  fs["device_name"] >> info.name;
  fs["serial_number"] >> info.serial_number;
  info.firmware_version = Version(std::string(fs["firmware_version"]));
  info.hardware_version = HardwareVersion(std::string(fs["hardware_version"]));
  info.spec_version = Version(std::string(fs["spec_version"]));
  info.lens_type = Type(std::string(fs["lens_type"]));
  info.imu_type = Type(std::string(fs["imu_type"]));
  fs["nominal_baseline"] >> info.nominal_baseline;
  fs.release();
  return info;
}

DeviceWriter::imu_params_t DeviceWriter::LoadImuParams(
    const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Failed to load file: " << filepath << std::endl;
  }
  imu_params_t params;
  fs["in_accel"] >> params.in_accel;
  fs["in_gyro"] >> params.in_gyro;
  fs["ex_left_to_imu"] >> params.ex_left_to_imu;
  fs.release();
  return params;
}


} // namespace tools

MYNTEYE_END_NAMESPACE
