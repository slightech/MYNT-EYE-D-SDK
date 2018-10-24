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
#include <iomanip>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include "mynteye/camera.h"
#include "dataset/dataset.h"
#include "mynteye/util/times.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char const *argv[]) {
  std::string dashes(80, '-');

  mynteye::Camera cam;

  mynteye::DeviceInfo dev_info;
  {
    std::vector<mynteye::DeviceInfo> dev_infos = cam.GetDevices();
    size_t n = dev_infos.size();
    if (n <= 0) {
      std::cerr << "Error: Device not found" << std::endl;
      return 1;
    }

    std::cout << dashes << std::endl;
    std::cout << "Index | Device Information" << std::endl;
    std::cout << dashes << std::endl;
    for (auto &&info : dev_infos) {
      std::cout << std::setw(5) << info.index << " | " << info << std::endl;
    }
    std::cout << dashes << std::endl;

    if (n <= 2) {
      dev_info = dev_infos[0];
      std::cout << "Auto select a device to open, index: 0"<< std::endl;
    } else {
      size_t i;
      std::cout << "Please select a device to open, index: ";
      std::cin >> i;
      std::cout << std::endl;
      if (i >= n) {
        std::cerr << "Error: Index out of range" << std::endl;
        return 1;
      }
      dev_info = dev_infos[i];
    }
  }

  {
    std::vector<mynteye::StreamInfo> color_infos;
    std::vector<mynteye::StreamInfo> depth_infos;
    cam.GetResolutions(dev_info.index, &color_infos, &depth_infos);

    std::cout << dashes << std::endl;
    std::cout << "Index | Color Stream Information" << std::endl;
    std::cout << dashes << std::endl;
    for (auto &&info : color_infos) {
      std::cout << std::setw(5) << info.index << " | " << info << std::endl;
    }
    std::cout << dashes << std::endl << std::endl;

    std::cout << dashes << std::endl;
    std::cout << "Index | Depth Stream Information" << std::endl;
    std::cout << dashes << std::endl;
    for (auto &&info : depth_infos) {
      std::cout << std::setw(5) << info.index << " | " << info << std::endl;
    }
    std::cout << dashes << std::endl << std::endl;
}

  std::cout << "Open device: " << dev_info.index << ", "
    << dev_info.name << std::endl << std::endl;

  // Warning: Color stream format MJPG doesn't work.
  mynteye::InitParams params(dev_info.index);
  params.depth_mode = mynteye::DepthMode::DEPTH_COLORFUL;
  // params.stream_mode = StreamMode::STREAM_1280x720;
  params.ir_intensity = 4;

  // output file path
  const char *outdir;
  if (argc >= 2) {
    outdir = argv[1];
  } else {
    outdir = "./dataset";
  }
  d1000_tools::Dataset dataset(outdir);

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);

  double t, fps = 0;
  cv::Mat color, depth;
  std::size_t imu_count = 0;
  auto &&time_beg = mynteye::times::now();
  for (;;) {
    auto image_color = cam.RetrieveImage(mynteye::ImageType::IMAGE_LEFT_COLOR);
    auto image_depth = cam.RetrieveImage(mynteye::ImageType::IMAGE_DEPTH);
    if (image_color.img) {
      cv::Mat color =
          image_color.img->To(mynteye::ImageFormat::COLOR_BGR)->ToMat();
      cv::imshow("color", color);

      dataset.SaveStreamData(image_color);
    }
    if (image_depth.img) {
      cv::Mat depth =
          image_depth.img->To(mynteye::ImageFormat::DEPTH_BGR)->ToMat();
      cv::imshow("depth", depth);
    }

    t = static_cast<double>(cv::getTickCount());
    auto &&motion_data = cam.RetrieveMotions();
    imu_count += motion_data.size();
    for (auto &&motion : motion_data) {
      dataset.SaveMotionData(motion);
    }
    std::cout << "\rSaved " << ", " << imu_count << " imus" << std::flush;


    char key = static_cast<char>(cv::waitKey(10));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }

    t = static_cast<double>(cv::getTickCount()) - t;
    fps = cv::getTickFrequency() / t;
  }
  std::cout << " to " << outdir << std::endl;
  auto &&time_end = mynteye::times::now();
  (void)(fps);

  cam.Close();

  float elapsed_ms =
      mynteye::times::count<mynteye::times::microseconds>(time_end - time_beg) *
      0.001f;
  std::cout << "Time beg: " << mynteye::times::to_local_string(time_beg)
    << ", end: " << mynteye::times::to_local_string(time_end)
    << ", cost: " << elapsed_ms << "ms" << std::endl;
  //  LOG(INFO) << "Img count: " << img_count
  //   << ", fps: " << (1000.f * img_count / elapsed_ms);
  std::cout << "Imu count: " << imu_count
    << ", hz: " << (1000.f * imu_count / elapsed_ms) << std::endl;

  cv::destroyAllWindows();
  return 0;
}
