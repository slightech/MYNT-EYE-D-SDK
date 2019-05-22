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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/visualization/cloud_viewer.h>
#pragma GCC diagnostic pop

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"
#include "util/pc_viewer.h"

#define CAMERA_FACTOR 1000.0

MYNTEYE_USE_NAMESPACE

int main(int argc, char const* argv[]) {
  // About warning in vtkOutputWindow with prebuilt version PCL on Windows.
  // Please see: Ugrade vtk api to 8.1 for 1.9,
  //   https://github.com/PointCloudLibrary/pcl/issues/2619
  // vtkObject::GlobalWarningDisplayOff();

  Camera cam;
  DeviceInfo dev_info;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  util::print_stream_infos(cam, dev_info.index);

  std::cout << "Open device: " << dev_info.index << ", "
      << dev_info.name << std::endl << std::endl;

  OpenParams params(dev_info.index);
  params.color_mode = ColorMode::COLOR_RECTIFIED;
  // Note: must set DEPTH_RAW to get raw depth values for points
  params.depth_mode = DepthMode::DEPTH_RAW;
  params.stream_mode = StreamMode::STREAM_1280x720;
  params.ir_intensity = 4;

  StreamMode stream_mode = params.stream_mode;

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  cv::namedWindow("color");

  auto stream_intrinsics = cam.GetStreamIntrinsics(stream_mode);

  CVPainter painter;
  PCViewer viewer(stream_intrinsics.left, CAMERA_FACTOR);
  util::Counter counter;
  cv::Mat color;
  cv::Mat depth;
  for (;;) {
    cam.WaitForStream();
    counter.Update();

    auto image_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
    auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
    if (image_color.img && color.empty()) {
      color = image_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
    }
    if (image_depth.img && depth.empty()) {
      depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();
    }

    if (color.empty() || depth.empty()) { continue; }

    viewer.Update(color, depth);

    painter.DrawSize(color, CVPainter::TOP_LEFT);
    painter.DrawStreamData(color, image_color, CVPainter::TOP_RIGHT);
    painter.DrawInformation(color, util::to_string(counter.fps()),
        CVPainter::BOTTOM_RIGHT);

    cv::imshow("color", color);

    color.release();
    depth.release();

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
    if (viewer.WasStopped()) {
      break;
    }
  }

  cam.Close();
  cv::destroyAllWindows();
  return 0;
}
