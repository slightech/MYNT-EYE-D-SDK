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

#include "mynteye/camera.h"
#include "mynteye/utils.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"

int main(int argc, char const* argv[]) {
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
  // params.color_mode = mynteye::ColorMode::COLOR_RECTIFIED;
  // params.depth_mode = mynteye::DepthMode::DEPTH_GRAY;
  params.depth_mode = mynteye::DepthMode::DEPTH_COLORFUL;
  // params.stream_mode = mynteye::StreamMode::STREAM_640x480;
  params.stream_mode = mynteye::StreamMode::STREAM_2560x720;
  params.ir_intensity = 4;
  params.framerate = 30;

  cam.EnableImageType(mynteye::ImageType::ALL);
  // cam.EnableImuProcessMode(mynteye::ProcessMode::ALL);
  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  cv::namedWindow("left color");
  cv::namedWindow("right color");
  cv::namedWindow("depth");

  mynteye::util::Counter counter;
  for (;;) {
    counter.Update();

    auto left_color = cam.RetrieveImage(mynteye::ImageType::IMAGE_LEFT_COLOR);
    auto right_color = cam.RetrieveImage(mynteye::ImageType::IMAGE_RIGHT_COLOR);
    auto image_depth = cam.RetrieveImage(mynteye::ImageType::IMAGE_DEPTH);
    if (left_color.img && right_color.img) {
      cv::Mat left = left_color.img->To(mynteye::ImageFormat::COLOR_BGR)->ToMat();
      cv::Mat right = right_color.img->To(mynteye::ImageFormat::COLOR_BGR)->ToMat();
      mynteye::util::draw(left, mynteye::util::to_string(counter.fps(), 5, 1),
          mynteye::util::TOP_RIGHT);
      cv::imshow("left color", left);
	  cv::imshow("right color", right);
    }
    if (image_depth.img) {
      cv::Mat depth = image_depth.img->To(mynteye::ImageFormat::DEPTH_BGR)->ToMat();
      cv::imshow("depth", depth);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
    cam.Wait();  // keep frequency
  }

  cam.Close();
  cv::destroyAllWindows();
  return 0;
}
