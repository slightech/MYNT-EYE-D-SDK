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

#include "mynteye/camera.h"
#include "mynteye/utils.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr cloud ( new PointCloud );
pcl::visualization::PCLVisualizer viewer("point cloud viewer");

// TODO replace the real camera param
const float camera_factor = 1000.0;

double camera_cx = 682.3;
double camera_cy = 254.9;
double camera_fx = 979.8;
double camera_fy = 942.8;

// show point cloud
void show_points(cv::Mat rgb, cv::Mat depth) {
  // loop the mat
  for (int m = 0; m < depth.rows; m++) {
    for (int n = 0; n < depth.cols; n++) {
      // get depth value at (m, n)
      std::uint16_t d = depth.ptr<std::uint16_t>(m)[n];
      // when d is equal 0 or 4096 means no depth
      if (d == 0 || d == 4096)
        continue;

      PointT p;

      // get point x y z
      p.z = static_cast<float>(d) / camera_factor;
      p.x = (n - camera_cx) * p.z / camera_fx;
      p.y = (m - camera_cy) * p.z / camera_fy;

      // get colors
      p.b = rgb.ptr<uchar>(m)[n * 3];
      p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
      p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

      // add point to cloud
      cloud->points.push_back(p);
    }
  }

  pcl::visualization::PointCloudColorHandlerRGBField<PointT>color(cloud);
  viewer.updatePointCloud<PointT>(cloud, color, "sample cloud");
  viewer.spinOnce();
  // clear points
  cloud->points.clear();
}

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
  params.depth_mode = DepthMode::DEPTH_RAW;
  params.stream_mode = StreamMode::STREAM_1280x720;
  params.ir_intensity = 4;

  // Enable what stream datas: left_color, right_color, depth
  cam.EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
  cam.EnableStreamData(ImageType::IMAGE_DEPTH);

  StreamMode stream_mode = params.stream_mode;

  cam.Open(params);

  auto streamIntrinsics = cam.GetStreamIntrinsics(stream_mode);

  camera_cx = streamIntrinsics.left.cx;
  camera_cy = streamIntrinsics.left.cy;
  camera_fx = streamIntrinsics.left.fx;
  camera_fy = streamIntrinsics.left.fy;

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  {
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    viewer.addPointCloud<PointT>(cloud, "sample cloud");
    viewer.setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    viewer.setSize(1280, 720);
  }

  cv::namedWindow("color");

  CVPainter painter;
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
      painter.DrawText(color, util::to_string(counter.fps()),
          CVPainter::BOTTOM_RIGHT);

      cv::Mat depth = image_depth.img->To(ImageFormat::DEPTH_RAW)
          ->ToMat();

      cv::imshow("color", color);

      show_points(color, depth);
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
