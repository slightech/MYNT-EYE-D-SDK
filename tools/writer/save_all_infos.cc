#include "mynteye/internal/camera_p.h"
#include "writer/device_writer.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  std::string dir{"config"};
  if (argc >= 2) {
    dir = argv[1];
  }

  auto &&device = std::make_shared<CameraPrivate>();
  if (!device)
    return 1;

  dir.append(MYNTEYE_OS_SEP "SN").append(device->GetInfo()->serial_number);

  tools::DeviceWriter writer(device);
  writer.SaveAllInfos(dir);

  return 0;
}
