#include <iostream>
#include <fstream>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  Camera cam;
  DeviceInfo dev_info;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  util::print_stream_infos(cam, dev_info.index);

  std::cout << "Open device: " << dev_info.index << ", "
    << dev_info.name << std::endl << std::endl;

  OpenParams params(dev_info.index);
  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::ofstream out("imu_params.params");

  bool in_ok, ex_ok;
  auto intrinsics = cam.GetMotionIntrinsics(&in_ok);
  if (in_ok) {
    std::cout << "Motion Intrinsics: {" << intrinsics << "}" << std::endl;
    out << "Motion Intrinsics: {" << intrinsics << "}" << std::endl;
  } else {
    std::cout << "This device not supported to get motion intrinsics." << std::endl;
  }
  auto extrinsics = cam.GetMotionExtrinsics(&ex_ok);
  if (ex_ok) {
    std::cout << "Motion Extrinsics left to imu: {" << extrinsics << "}" << std::endl;
    out << "Motion Extrinsics left to imu: {" << extrinsics << "}" << std::endl;
  } else {
    std::cout << "This device not supported to get motion extrinsics." << std::endl;
  }

  cam.Close();
  out.close();

  std::cout << std::endl;
  if (in_ok || ex_ok) {
    std::cout << "Imu params saved to imu_params.params in current folder." << std::endl;
  }
  return 0;
}
