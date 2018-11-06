#include "mynteye/internal/camera_p.h"

#include "writer/device_writer.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  const char *filepath;
  if (argc >= 2) {
    filepath = argv[1];
  } else {
    std::cout << "Usage: ./device_info_writer <filepath>" << std::endl;
    return 2;
  }

  auto &&device = std::make_shared<CameraPrivate>();
  if (!device)
    return 1;

  tools::DeviceWriter writer(device);
  writer.WriteDeviceInfo(filepath);

  return 0;
}
