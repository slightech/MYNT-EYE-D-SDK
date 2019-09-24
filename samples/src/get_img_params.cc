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

  std::ofstream out("image_params.params");

  bool in_ok, ex_ok;
  auto vga_intrinsics = cam.GetStreamIntrinsics(StreamMode::STREAM_1280x480, &in_ok);
  auto vga_extrinsics = cam.GetStreamExtrinsics(StreamMode::STREAM_1280x480, &ex_ok);
  if (in_ok && ex_ok) {
    std::cout << "VGA Intrinsics left: {" << vga_intrinsics.left << "}" << std::endl;
    std::cout << "VGA Intrinsics right: {" << vga_intrinsics.right << "}" << std::endl;
    std::cout << "VGA Extrinsics left to right: {" << vga_extrinsics << "}" << std::endl;

    out << "VGA Intrinsics left: {" << vga_intrinsics.left << "}" << std::endl;
    out << "VGA Intrinsics right: {" << vga_intrinsics.right << "}" << std::endl;
    out << "VGA Extrinsics left to right: {" << vga_extrinsics << "}" << std::endl;
  } else {
    std::cout << "This device not supported to get vga image params." << std::endl;
  }

  auto hd_intrinsics = cam.GetStreamIntrinsics(StreamMode::STREAM_2560x720, &in_ok);
  auto hd_extrinsics = cam.GetStreamExtrinsics(StreamMode::STREAM_2560x720, &ex_ok);
  if (in_ok && ex_ok) {
    std::cout << "HD Intrinsics left: {" << hd_intrinsics.left << "}" << std::endl;
    std::cout << "HD Intrinsics right: {" << hd_intrinsics.right << "}" << std::endl;
    std::cout << "HD Extrinsics left to right: {" << hd_extrinsics << "}" << std::endl;

    out << "HD Intrinsics left: {" << hd_intrinsics.left << "}" << std::endl;
    out << "HD Intrinsics right: {" << hd_intrinsics.right << "}" << std::endl;
    out << "HD Extrinsics left to right: {" << hd_extrinsics << "}" << std::endl;
  } else {
    std::cout << "This device not supported to get hd image params." << std::endl;
  }

  std::cout << "If you cant't have a clear understanding of the info,"
               "you can read the ROS-doc (http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html)"  // NOLINT
               " to learn more.";

  cam.Close();
  out.close();
  std::cout << std::endl;
  if (in_ok && ex_ok) {
    std::cout << "Image params saved to image_params.params in current folder." << std::endl;
  }
  return 0;
}
