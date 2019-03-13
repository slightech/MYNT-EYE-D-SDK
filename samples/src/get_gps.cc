// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"
#include "mynteyed/util/rate.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char const* argv[]) {
  Camera cam;
  DeviceInfo dev_info;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  util::print_stream_infos(cam, dev_info.index);

  std::cout << "Open device: " << dev_info.index << ", "
      << dev_info.name << std::endl << std::endl;

  if (!cam.IsLocationDatasSupported() ||
      !cam.IsDistanceDatasSupported()) {
    std::cerr << "Error: GPS and Ultrasonic is not supported on your device." << std::endl;
    return 1;
  }

  // Warning: Color stream format MJPG doesn't work.
  OpenParams params(dev_info.index);
  cam.Open(params);

  // Enable this will cache the motion datas until you get them
  cam.EnableLocationDatas();
  cam.EnableDistanceDatas();

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  Rate rate(params.framerate);
  util::Counter counter;
  for (;;) {
    counter.Update();

    auto location_datas = cam.GetLocationDatas();
    if (location_datas.size() > 0) {
      std::cout << "GPS count: " << location_datas.size() << std::endl;
      for (auto data : location_datas) {
        if (data.gps) {
          counter.IncrGPSCount();
          std::cout << "[GPS] device_time: " << data.gps->device_time
            << ", latitude: " << data.gps->latitude
            << ", longitude: " << data.gps->longitude
            << ", latitude_degree: " << data.gps->latitude_degree
            << ", latitude_cent: " << data.gps->latitude_cent
            << ", latitude_second: " << data.gps->latitude_second
            << ", longitude_degree: " << data.gps->longitude_degree
            << ", longitude_cent: " << data.gps->longitude_cent
            << ", longitude_second: " << data.gps->longitude_second
            << ", NS: " << data.gps->NS
            << ", EW: " << data.gps->EW
            << ", year: " << unsigned(data.gps->year)
            << ", month: " << unsigned(data.gps->month)
            << ", day: " << unsigned(data.gps->day)
            << ", hour: " << unsigned(data.gps->hour)
            << ", minute: " << unsigned(data.gps->minute)
            << ", second: " << unsigned(data.gps->second) << std::endl;
        }
        std::cout << std::endl;
      }
    }

    if (_kbhit()) break;

    rate.Sleep();
  }

  cam.Close();

  counter.PrintCountInfo();
  return 0;
}
