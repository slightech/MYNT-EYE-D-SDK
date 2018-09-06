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

// PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "mynteye/camera.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"

using namespace std;
using namespace mynteye;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr cloud ( new PointCloud );
pcl::visualization::PCLVisualizer viewer("point cloud viewer");

// TODO replace the real camera param
const float camera_factor = 1000.0;

const double camera_cx = 682.3;
const double camera_cy = 254.9;
const double camera_fx = 979.8;
const double camera_fy = 942.8;

// show point cloud
void show_points(cv::Mat rgb, cv::Mat depth) {
  // loop the mat
  for (int m = 0; m < depth.rows; m++) {
    for (int n = 0; n < depth.cols; n++) {
      // get depth value at (m, n)
      unsigned short d = depth.ptr<unsigned short>(m)[n];
      // when d is equal 0 or 4096 means no depth
      if (d == 0 || d == 4096)
        continue;

      PointT p;

      // get point x y z
      p.z = float(d) / camera_factor;
      p.x = (n - camera_cx) * p.z / camera_fx;
      p.y = (m - camera_cy) * p.z / camera_fy;

      // get colors
      p.b = rgb.ptr<uchar>(m)[n * 3];
      p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
      p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

      // add point to cloud
      cloud->points.push_back( p );
    }
  }

  pcl::visualization::PointCloudColorHandlerRGBField<PointT> color (cloud);
  viewer.updatePointCloud<PointT>(cloud, color, "sample cloud");
  viewer.spinOnce();
  // clear points
  cloud->points.clear();
}

int main(int argc, char const *argv[]) {
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
  params.depth_mode = DepthMode::DEPTH_NON_16UC1;
  // params.stream_mode = StreamMode::STREAM_1280x720;
  params.ir_intensity = 4;

  cam.Open(params);

  cout << endl;
  if (!cam.IsOpened()) {
    cerr << "Error: Open camera failed" << endl;
    return 1;
  }
  cout << "Open device success" << endl << endl;

  cout << "Press ESC/Q on Windows to terminate" << endl;

  {
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    viewer.addPointCloud<PointT>(cloud, "sample cloud");
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);
    viewer.setSize(1280, 720);
  }

  cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);

  cv::Mat color, depth;
  util::Counter counter;
  for (;;) {
    counter.Update();

    if (cam.RetrieveImage(color, depth) == ErrorCode::SUCCESS) {
      util::draw(color, util::to_string(counter.fps(), 5, 1), util::TOP_RIGHT);
      cv::imshow("color", color);
      cv::imshow("depth", depth);
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
