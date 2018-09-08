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

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"

using namespace std;
using namespace mynteye;

int main(int argc, char const* argv[]) {
  Camera cam;
  DeviceInfo dev_info;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  util::print_stream_infos(cam, dev_info.index);

  cout << "Open device: " << dev_info.index << ", "
      << dev_info.name << endl << endl;

  // Warning: Color stream format MJPG doesn't work.
  InitParams params(dev_info.index);
  // params.depth_mode = DepthMode::DEPTH_GRAY;
  params.depth_mode = DepthMode::DEPTH_COLORFUL;
  // params.stream_mode = StreamMode::STREAM_640x480;
  params.ir_intensity = 4;

  cam.Open(params);

  cout << endl;
  if (!cam.IsOpened()) {
    cerr << "Error: Open camera failed" << endl;
    return 1;
  }
  cout << "Open device success" << endl << endl;

  cout << "Press ESC/Q on Windows to terminate" << endl;

  cv::namedWindow("color");
  cv::namedWindow("depth");

  util::Counter counter;
  for (;;) {
    counter.Update();

    auto image_color = cam.RetrieveImage(ImageType::IMAGE_COLOR);
    auto image_depth = cam.RetrieveImage(ImageType::IMAGE_DEPTH);
    if (image_color && image_depth) {
      cv::Mat color = image_color->To(ImageFormat::COLOR_BGR)->ToMat();
      cv::Mat depth = image_depth->To(ImageFormat::DEPTH_BGR)->ToMat();
      util::draw(color, util::to_string(counter.fps(), 5, 1), util::TOP_RIGHT);
      cv::imshow("color", color);
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
