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

#ifdef WITH_OPENCV2
#include <opencv2/highgui/highgui.hpp>
#else
#include <opencv2/imgcodecs/imgcodecs.hpp>
#endif

// #include <pcl/common/common_headers.h>
#include <pcl/io/ply_io.h>

#include <cmath>
#include <string>

#include "mynteyed/util/files.h"
#include "mynteyed/util/times.h"

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

PCViewer::PCViewer(const MYNTEYE_NAMESPACE::CameraIntrinsics& cam_in,
    float cam_factor)
  : viewer_(nullptr), cam_in_(cam_in), cam_factor_(cam_factor),
    generating_(false), running_(false) {
  Start();
}

PCViewer::~PCViewer() {
  Stop();
  if (viewer_) {
    // viewer_->saveCameraParameters("pcl_camera_params.txt");
    viewer_->close();
    viewer_ == nullptr;
  }
}

bool PCViewer::Update(const cv::Mat &rgb, const cv::Mat& depth) {
  if (generating_) return false;
  {
    std::lock_guard<std::mutex> _(mutex_);
    if (generating_) return false;
    generating_ = true;
    rgb_ = rgb.clone();
    depth_ = depth.clone();
  }
  condition_.notify_one();
  return true;
}

bool PCViewer::UpdateDirectly(const cv::Mat &rgb, const cv::Mat& depth) {
  pointcloud_t::Ptr cloud(new pointcloud_t);
  ConvertToPointCloud(rgb, depth, cloud);
  Update(cloud);
  return true;
}

void PCViewer::Update(pointcloud_t::ConstPtr cloud) {
  if (viewer_ == nullptr) {
    viewer_ = CustomColorVis(cloud, cam_in_.width, cam_in_.height);
    viewer_->registerKeyboardCallback(
      boost::bind(&PCViewer::KeyboardCallback, this, _1));
    std::cout << std::endl
        << "Press 'Space' on Viewer to save *.ply, *.png" << std::endl;
  }
  pcl::visualization::PointCloudColorHandlerRGBField<point_t>color(cloud);
  viewer_->updatePointCloud<point_t>(cloud, color, "points");
  viewer_->spinOnce();
  cloud_ = cloud;
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

void PCViewer::Start() {
  if (running_) return;
  {
    std::lock_guard<std::mutex> _(mutex_);
    if (running_) return;
    running_ = true;
  }
  thread_ = std::thread(&PCViewer::Run, this);
}

void PCViewer::Stop() {
  if (!running_) return;
  {
    std::lock_guard<std::mutex> _(mutex_);
    if (!running_) return;
    running_ = false;
    generating_ = true;
  }
  condition_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void PCViewer::Run() {
  while (running_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return generating_; });
      if (!running_) break;
    }

    pointcloud_t::Ptr cloud(new pointcloud_t);
    ConvertToPointCloud(rgb_, depth_, cloud);
    Update(cloud);

    {
      std::lock_guard<std::mutex> _(mutex_);
      generating_ = false;
    }
  }
}

void PCViewer::KeyboardCallback(
    const pcl::visualization::KeyboardEvent& event) {
  MYNTEYE_USE_NAMESPACE
  if (event.getKeySym() == "space" && event.keyDown()) {
    if (save_dir_.empty()) {
      auto dir = times::to_local_string(times::now(), "%Y%m%d%H%M%S", 0);
      if (files::mkdir(dir)) {
        save_dir_ = dir + MYNTEYE_OS_SEP;
      } else {
        std::cout << "Create directory failed: " << save_dir_ << std::endl;
      }
    }
    static int count = 1;
    {
      std::stringstream ss;
      ss << save_dir_ << "pointcloud-" << count << ".ply";
      std::string filename = ss.str();
      int ret = 0;
      if (cloud_ && (ret = pcl::io::savePLYFileBinary(filename, *cloud_))
          == 0) {
        std::cout << filename  << " saved" << std::endl;
      } else {
        std::cout << filename << " save failed: " << ret << std::endl;
      }
    }
    {
      std::stringstream ss;
      ss << save_dir_ << "image-" << count << ".png";
      std::string filename = ss.str();
      cv::imwrite(filename, rgb_);
        std::cout << filename  << " saved" << std::endl;
    }
    {
      std::stringstream ss;
      ss << save_dir_ << "depth-" << count << ".png";
      std::string filename = ss.str();
      cv::imwrite(filename, depth_);
        std::cout << filename  << " saved" << std::endl;
    }
    ++count;
  }
}

