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
#include "util/pc_viewer.h"

// #include <pcl/common/common_headers.h>

#include <cmath>

std::shared_ptr<pcl::visualization::PCLVisualizer> CustomColorVis(
    PCViewer::pointcloud_t::ConstPtr cloud, int xw, int yw) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->addPointCloud<PCViewer::point_t>(cloud, "points");
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer->setSize(xw, yw);
  return (viewer);
}

PCViewer::PCViewer(const mynteye::CameraIntrinsics& cam_in, float cam_factor)
  : viewer_(nullptr), cam_in_(cam_in), cam_factor_(cam_factor) {
}

PCViewer::~PCViewer() {
  if (viewer_) {
    // viewer_->saveCameraParameters("pcl_camera_params.txt");
    viewer_->close();
    viewer_ == nullptr;
  }
}

void PCViewer::Update(const cv::Mat &rgb, const cv::Mat& depth) {
  pointcloud_t::Ptr cloud(new pointcloud_t);
  ConvertToPointCloud(rgb, depth, cloud);
  Update(cloud);
}

void PCViewer::Update(pointcloud_t::ConstPtr cloud) {
  if (viewer_ == nullptr) {
    viewer_ = CustomColorVis(cloud, cam_in_.width, cam_in_.height);
  }
  pcl::visualization::PointCloudColorHandlerRGBField<point_t>color(cloud);
  viewer_->updatePointCloud<point_t>(cloud, color, "points");
  viewer_->spinOnce();
}

bool PCViewer::WasVisual() const {
  return viewer_ != nullptr;
}

bool PCViewer::WasStopped() const {
  return viewer_ != nullptr && viewer_->wasStopped();
}

void PCViewer::ConvertToPointCloud(
    const cv::Mat &rgb, const cv::Mat& depth, pointcloud_t::Ptr cloud) {
  // loop the mat
  for (int m = 0; m < depth.rows; m++) {
    for (int n = 0; n < depth.cols; n++) {
      // get depth value at (m, n)
      std::uint16_t d = depth.ptr<std::uint16_t>(m)[n];
      // when d is equal 0 or 4096 means no depth
      if (d == 0 || d == 4096)
        continue;

      point_t p;

      // get point x y z
      p.z = static_cast<float>(d) / cam_factor_;
      p.x = (n - cam_in_.cx) * p.z / cam_in_.fx;
      p.y = (m - cam_in_.cy) * p.z / cam_in_.fy;

      // get colors
      p.b = rgb.ptr<uchar>(m)[n * 3];
      p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
      p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

      // add point to cloud
      cloud->points.push_back(p);
    }
  }
}
