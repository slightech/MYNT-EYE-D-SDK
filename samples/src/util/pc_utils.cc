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
#include "util/pc_utils.h"

#include <string>
#include <vector>

#include <pcl/io/ply_io.h>

MYNTEYE_BEGIN_NAMESPACE

namespace util {

inline
CameraIntrinsics get_camera_intrinsics(const Camera &camera) {
  auto stream_mode = camera.GetOpenParams().stream_mode;
  return camera.GetStreamIntrinsics(stream_mode).left;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_point_cloud(
    Camera *camera, float cam_factor,
    std::vector<std::shared_ptr<BaseFilter>> filters) {
  static auto cam_in = get_camera_intrinsics(*camera);

  cv::Mat color;
  auto image_color = camera->GetStreamData(ImageType::IMAGE_LEFT_COLOR);
  if (image_color.img) {
    color = image_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
  }

  cv::Mat depth;
  auto image_depth = camera->GetStreamData(ImageType::IMAGE_DEPTH);
  if (image_depth.img) {
    depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();
  }

  if (color.empty() || depth.empty()) { return nullptr; }
  for (size_t i=0; i< filters.size(); i++) {
    filters[i]->ProcessFrame(image_depth.img, image_depth.img);
  }

  return get_point_cloud(color, depth, cam_in, cam_factor);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_point_cloud(
    Camera *camera, float cam_factor) {
  static auto cam_in = get_camera_intrinsics(*camera);

  cv::Mat color;
  auto image_color = camera->GetStreamData(ImageType::IMAGE_LEFT_COLOR);
  if (image_color.img) {
    color = image_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
  }

  cv::Mat depth;
  auto image_depth = camera->GetStreamData(ImageType::IMAGE_DEPTH);
  if (image_depth.img) {
    depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();
  }

  if (color.empty() || depth.empty()) { return nullptr; }

  return get_point_cloud(color, depth, cam_in, cam_factor);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_point_cloud(
    const cv::Mat &rgb, const cv::Mat& depth,
    const CameraIntrinsics& cam_in, float cam_factor) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  for (int m = 0; m < depth.rows; m++) {
    for (int n = 0; n < depth.cols; n++) {
      std::uint16_t d = depth.ptr<std::uint16_t>(m)[n];
      if (d == 0) continue;
      pcl::PointXYZRGBA p;
      p.z = static_cast<float>(d) / cam_factor;
      p.x = (n - cam_in.cx) * p.z / cam_in.fx;
      p.y = (m - cam_in.cy) * p.z / cam_in.fy;
      p.b = rgb.ptr<uchar>(m)[n * 3];
      p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
      p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
      cloud->points.push_back(p);
    }
  }
  return cloud;
}

// PCViewer

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

PCViewer::PCViewer(int view_width, int view_height)
  : viewer_(nullptr), view_width_(view_width), view_height_(view_height) {
}

PCViewer::~PCViewer() {
  if (viewer_) {
    // viewer_->saveCameraParameters("pcl_camera_params.txt");
    viewer_->close();
    viewer_ == nullptr;
  }
}

void PCViewer::Update(pointcloud_t::ConstPtr cloud) {
  if (viewer_ == nullptr) {
    viewer_ = CustomColorVis(cloud, view_width_, view_height_);
    viewer_->registerKeyboardCallback(
      boost::bind(&PCViewer::KeyboardCallback, this, _1));
    std::cout << std::endl
        << "Press 'Space' on Viewer to save *.ply, 'J' to capture screenshot"
        << std::endl;
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

void PCViewer::KeyboardCallback(
    const pcl::visualization::KeyboardEvent& event) {
  if (event.getKeySym() == "space" && event.keyDown()) {
    static int count = 1;
    std::stringstream ss;
    ss << "pointcloud-" << (count++) << ".ply";
    std::string filename = ss.str();
    int ret = 0;
    if (cloud_ && (ret = pcl::io::savePLYFileBinary(filename, *cloud_)) == 0) {
      std::cout << "PLY file (" << filename << ") successfully saved."
          << std::endl;
    } else {
      std::cout << "PLY file save failed: " << ret << std::endl;
    }
  }
}

}  // namespace util

MYNTEYE_END_NAMESPACE
