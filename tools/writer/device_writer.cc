#include "writer/device_writer.h"

#include <vector>

#include <opencv2/core/core.hpp>

#include "mynteyed/util/files.h"

MYNTEYE_BEGIN_NAMESPACE

namespace tools {

DeviceWriter::DeviceWriter(std::shared_ptr<Camera> device)
    : device_(device) {
}

DeviceWriter::~DeviceWriter() {
}

bool DeviceWriter::WriteDescriptors(const device_desc_t &desc) {
  auto &&dev_desc = device_->GetDescriptors();
  if (nullptr == dev_desc) {
    std::cerr << "\nDevice was not supported to write." << std::endl;
    return false;
  }
  dev_desc->lens_type = Type(desc.lens_type);
  dev_desc->imu_type = Type(desc.imu_type);
  dev_desc->nominal_baseline = desc.nominal_baseline;
  if (device_->WriteDeviceFlash(dev_desc.get(), nullptr, nullptr)) {
    std::cout << std::endl;
    std::cout << "Write device info success" << std::endl;
    std::cout << "Device info: {name: " << dev_desc->name
      << " serial_number: " << dev_desc->serial_number
      << " firmware_version: " << dev_desc->firmware_version.to_string()
      << " hardware_version: " << dev_desc->hardware_version.to_string()
      << " spec_version: " << dev_desc->spec_version.to_string()
      << " lens_type: " << dev_desc->lens_type.to_string()
      << " imu_type: " << dev_desc->imu_type.to_string()
      << " nominal_baseline: " << dev_desc->nominal_baseline
      << "}" << std::endl;
    std::cout << std::endl;
    return true;
  } else {
    std::cerr << "Write device info failed" << std::endl;
    return false;
  }
}

bool DeviceWriter::WriteDescriptors(const std::string &filepath) {
  return WriteDescriptors(LoadDescriptors(filepath));
}

bool DeviceWriter::WriteImuParams(const imu_params_t &params) {
  auto &&dev_desc = device_->GetDescriptors();
  if (nullptr == dev_desc) {
    std::cerr << "\nDevice was not supported to write." << std::endl;
    return false;
  }
  if (device_->WriteDeviceFlash(dev_desc.get(),
        const_cast<imu_params_t *>(&params), &dev_desc->spec_version)) {
    std::cout << std::endl
        << "Write imu params success" << std::endl
        << "Imu intrinsics accel: {" << params.in_accel << "}" << std::endl
        << "Imu intrinsics gyro: {" << params.in_gyro << "}" << std::endl
        << "Imu extrinsics left to imu: {" << params.ex_left_to_imu << "}"
        << std::endl << std::endl;
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

}  // namespace

bool DeviceWriter::SaveDescriptors(
    const device_desc_t &desc, const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cout << "Failed to save file: " << filepath << std::endl;
    return false;
  }
  fs << "device_name" << desc.name;
  fs << "serial_number" << desc.serial_number;
  fs << "firmware_version" << desc.firmware_version.to_string();
  fs << "hardware_version" << desc.hardware_version.to_string();
  fs << "spec_version" << desc.spec_version.to_string();
  fs << "lens_type" << desc.lens_type.to_string();
  fs << "imu_type" << desc.imu_type.to_string();
  fs << "nominal_baseline" << desc.nominal_baseline;
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

void DeviceWriter::SaveAllDatas(const std::string &dir) {
  if (!files::mkdir(dir)) {
    std::cout << "Create directory failed" << std::endl;
  }
  auto descs = device_->GetDescriptors();
  if (descs) {
    SaveDescriptors(*descs, dir + MYNTEYE_OS_SEP "device.info");
  } else {
    std::cerr << "Get Descriptors failed" << std::endl;
  }

  bool ok;
  auto &&m_in = device_->GetMotionIntrinsics(&ok);
  if (ok) {
    SaveImuParams({
      false, m_in.accel, m_in.gyro,
      device_->GetMotionExtrinsics(),
    }, dir + MYNTEYE_OS_SEP "imu.params");
  } else {
    std::cerr << "Get imu params failed" << std::endl;
  }

  std::cout << "\nSaved to " << dir << " folder" << std::endl;
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

}  // namespace

DeviceWriter::device_desc_t DeviceWriter::LoadDescriptors(
    const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Failed to load file: " << filepath << std::endl;
  }
  device_desc_t desc;
  desc.name = std::string(fs["device_name"]);
  desc.serial_number = std::string(fs["serial_number"]);
  desc.firmware_version = Version(std::string(fs["firmware_version"]));
  desc.hardware_version = HardwareVersion(std::string(fs["hardware_version"]));
  desc.spec_version = Version(std::string(fs["spec_version"]));
  desc.lens_type = Type(std::string(fs["lens_type"]));
  desc.imu_type = Type(std::string(fs["imu_type"]));
  fs["nominal_baseline"] >> desc.nominal_baseline;
  fs.release();
  return desc;
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

bool DeviceWriter::HidFirmwareUpdate(const char* filepath) {
  return device_->HidFirmwareUpdate(filepath);
}

}  // namespace tools

MYNTEYE_END_NAMESPACE
