#include "writer/device_writer.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  const char *filepath;
  if (argc >= 2) {
    filepath = argv[1];
  } else {
    std::cout << "Usage: ./auxiliary_firmware_update <filepath>" << std::endl;
    return 2;
  }

  auto &&device = std::make_shared<Camera>();

  tools::DeviceWriter writer(device);
  writer.AuxiliaryChipFirmwareUpdate(filepath);

  return 0;
}
