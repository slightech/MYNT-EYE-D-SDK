#include <iostream>
#include <fstream>

#include "mynteye/camera.h"
#include "mynteye/utils.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  mynteye::Camera cam;
  mynteye::DeviceInfo dev_info;
  if (!mynteye::util::select(cam, &dev_info)) {
    return 1;
  }
  mynteye::util::print_stream_infos(cam, dev_info.index);

  std::cout << "Open device: " << dev_info.index << ", "
    << dev_info.name << std::endl << std::endl;

    // Warning: Color stream format MJPG doesn't work.
  mynteye::OpenParams params(dev_info.index);
  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::ofstream out("image_params.params");

  auto vga_intrinsics = cam.GetStreamIntrinsics(StreamMode::STREAM_1280x480);
  auto vga_extrinsics = cam.GetStreamExtrinsics(StreamMode::STREAM_1280x480);
  std::cout << "VGA Intrinsics left: {" << vga_intrinsics.left << "}" << std::endl;
  std::cout << "VGA Intrinsics right: {" << vga_intrinsics.right << "}" << std::endl;
  std::cout << "VGA Extrinsics left to right: {" << vga_extrinsics << "}" << std::endl;

  out << "VGA Intrinsics left: {" << vga_intrinsics.left << "}" << std::endl;
  out << "VGA Intrinsics right: {" << vga_intrinsics.right << "}" << std::endl;
  out << "VGA Extrinsics left to right: {" << vga_extrinsics << "}" << std::endl;

  auto hd_intrinsics = cam.GetStreamIntrinsics(StreamMode::STREAM_2560x720);
  auto hd_extrinsics = cam.GetStreamExtrinsics(StreamMode::STREAM_2560x720);
  std::cout << "HD Intrinsics left: {" << hd_intrinsics.left << "}" << std::endl;
  std::cout << "HD Intrinsics right: {" << hd_intrinsics.right << "}" << std::endl;
  std::cout << "HD Extrinsics left to right: {" << hd_extrinsics << "}" << std::endl;

  out << "HD Intrinsics left: {" << hd_intrinsics.left << "}" << std::endl;
  out << "HD Intrinsics right: {" << hd_intrinsics.right << "}" << std::endl;
  out << "HD Extrinsics left to right: {" << hd_extrinsics << "}" << std::endl;

  cam.Close();
  out.close();
  std::cout << std::endl;
  std::cout << "Image params saved to image_params.params in current folder." << std::endl;
  return 0;
}
