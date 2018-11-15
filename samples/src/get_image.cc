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

  OpenParams params(dev_info.index);
  {
    // Framerate: 10(default), [0,60], [0,30](STREAM_2560x720)
    params.framerate = 30;

    // Color mode: raw(default), rectified
    // params.color_mode = ColorMode::COLOR_RECTIFIED;

    // Depth mode: colorful(default), gray, raw
    // params.depth_mode = DepthMode::DEPTH_GRAY;

    // Stream mode: left color only
    // params.stream_mode = StreamMode::STREAM_640x480;  // vga
    // params.stream_mode = StreamMode::STREAM_1280x720;  // hd
    // Stream mode: left+right color
    // params.stream_mode = StreamMode::STREAM_1280x480;  // vga
    params.stream_mode = StreamMode::STREAM_2560x720;  // hd

    // Auto-exposure: true(default), false
    // params.state_ae = false;

    // Auto-white balance: true(default), false
    // params.state_awb = false;

    // Infrared intensity: 0(default), [0,6]
    params.ir_intensity = 4;
  }

  // Enable what process logics
  // cam.EnableProcessMode(ProcessMode::PROC_IMU_ALL);

  // Enable what stream datas: left_color, right_color, depth
  cam.EnableStreamData(ImageType::IMAGE_ALL);

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

  util::Counter counter;
  for (;;) {
    counter.Update();

    auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
    if (left_color.img) {
      cv::Mat left = left_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
      util::draw(left, util::to_string(counter.fps(), 5, 1),
          util::TOP_RIGHT);
      cv::imshow("left color", left);
      // std::cout << "left frame id: " << left_color.img->frame_id()
      //     << std::endl;
    }

    if (util::is_right_color_supported(params.stream_mode)) {
      auto right_color = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
      if (right_color.img) {
        cv::Mat right = right_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        cv::imshow("right color", right);
        // std::cout << "right frame id: " << right_color.img->frame_id()
        //     << std::endl;
      }
    }

    auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
    if (image_depth.img) {
      cv::Mat depth = image_depth.img->To(ImageFormat::DEPTH_BGR)->ToMat();
      cv::imshow("depth", depth);
      // std::cout << "depth frame id: " << image_depth.img->frame_id()
      //     << std::endl;
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  cam.Close();
  cv::destroyAllWindows();
  return 0;
}
