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
#include <pcl/visualization/cloud_viewer.h>

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

CameraIntrinsics get_default_camera_intrinsics(const StreamMode& mode);

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
  params.depth_mode = DepthMode::DEPTH_RAW;
  params.stream_mode = StreamMode::STREAM_1280x720;
  params.ir_intensity = 4;

  // Enable what stream datas: left_color, right_color, depth
  cam.EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
  cam.EnableStreamData(ImageType::IMAGE_DEPTH);

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

  bool ok;
  auto stream_intrinsics = cam.GetStreamIntrinsics(stream_mode, &ok);
  CameraIntrinsics in = ok ? stream_intrinsics.left
      : get_default_camera_intrinsics(stream_mode);

  CVPainter painter;
  PCViewer viewer(in, CAMERA_FACTOR);
  util::Counter counter;
  for (;;) {
    counter.Update();

    auto image_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
    auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
    if (image_color.img && image_depth.img) {
      cv::Mat color = image_color.img->To(ImageFormat::COLOR_BGR)
          ->ToMat();
      painter.DrawSize(color, CVPainter::TOP_LEFT);
      painter.DrawStreamData(color, image_color, CVPainter::TOP_RIGHT);
      painter.DrawInformation(color, util::to_string(counter.fps()),
          CVPainter::BOTTOM_RIGHT);

      cv::Mat depth = image_depth.img->To(ImageFormat::DEPTH_RAW)
          ->ToMat();

      cv::imshow("color", color);

      viewer.Update(color, depth);
    }

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

CameraIntrinsics get_default_camera_intrinsics(const StreamMode& mode) {
  // {w, h, fx, fy, cx, cy, coeffs[5]{k1,k2,p1,p2,k3}}
  switch (mode) {
    case StreamMode::STREAM_640x480:
      return {640, 480, 979.8, 942.8, 682.3 / 2, 254.9, {0, 0, 0, 0, 0}};
    case StreamMode::STREAM_1280x480:
      return {640, 480, 979.8, 942.8, 682.3, 254.9, {0, 0, 0, 0, 0}};
    case StreamMode::STREAM_1280x720:
      return {640, 480, 979.8, 942.8, 682.3, 254.9 * 2, {0, 0, 0, 0, 0}};
    case StreamMode::STREAM_2560x720:
      return {640, 480, 979.8, 942.8, 682.3 * 2, 254.9 * 2, {0, 0, 0, 0, 0}};
    default:
      return {640, 480, 979.8, 942.8, 682.3, 254.9, {0, 0, 0, 0, 0}};
  }
}