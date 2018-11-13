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
#include "mynteye/utils.h"
#include "mynteye/util/times.h"

#include "dataset/dataset.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char const *argv[]) {
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
  params.depth_mode = mynteye::DepthMode::DEPTH_COLORFUL;
  params.stream_mode = StreamMode::STREAM_2560x720;
  params.ir_intensity = 4;
  params.framerate = 30;

  // output file path
  const char *outdir = nullptr;
  if (argc >= 2) {
    outdir = argv[1];
  } else {
    outdir = "./dataset";
  }
  tools::Dataset dataset(outdir);

  cam.EnableImageType(mynteye::ImageType::ALL);
  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  cv::namedWindow("color", cv::WINDOW_AUTOSIZE);

  double t, fps = 0;
  std::size_t imu_count = 0;
  std::size_t img_count = 0;
  auto &&time_beg = mynteye::times::now();
  for (;;) {
    auto &&left_color = cam.RetrieveImages(mynteye::ImageType::IMAGE_LEFT_COLOR);
    auto &&right_color = cam.RetrieveImages(mynteye::ImageType::IMAGE_RIGHT_COLOR);
    img_count += left_color.size();
    if (!left_color.empty() && !right_color.empty()) {
      auto &&left = left_color.back();
      auto &&right = right_color.back();
      cv::Mat image_left =
        left.img->To(mynteye::ImageFormat::COLOR_BGR)->ToMat();
      cv::Mat image_right =
        right.img->To(mynteye::ImageFormat::COLOR_BGR)->ToMat();

      cv::Mat color;
      cv::hconcat(image_left, image_right, color);
      cv::imshow("color", color);
    }

    t = static_cast<double>(cv::getTickCount());
    auto &&motion_data = cam.RetrieveMotions();
    imu_count += motion_data.size();

    {
      for (auto &&left : left_color) {
        dataset.SaveStreamData(
            mynteye::ImageType::IMAGE_LEFT_COLOR, left);
      }

      for (auto &&right : right_color) {
        dataset.SaveStreamData(
            mynteye::ImageType::IMAGE_RIGHT_COLOR, right);
      }

      for (auto &&motion : motion_data) {
        dataset.SaveMotionData(motion);
      }

      std::cout << "\rSaved " << img_count << " imgs" <<
        ", " << imu_count << " imus" << std::flush;
    }

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
  std::cout << "Img count: " << img_count
    << ", fps: " << (1000.f * img_count / elapsed_ms) << std::endl;
  std::cout << "Imu count: " << imu_count
    << ", hz: " << (1000.f * imu_count / elapsed_ms) << std::endl;

  cv::destroyAllWindows();
  return 0;
}
