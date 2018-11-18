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
#ifndef MYNTEYE_SAMPLES_PC_VIEWER_H_  // NOLINT
#define MYNTEYE_SAMPLES_PC_VIEWER_H_
#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <mynteye/types.h>

class PCViewer {
 public:
  using point_t = pcl::PointXYZRGBA;
  using pointcloud_t = pcl::PointCloud<point_t>;

  PCViewer(const mynteye::CameraIntrinsics& cam_in, float cam_factor);
  ~PCViewer();

  void Update(const cv::Mat &rgb, const cv::Mat& depth);

  void Update(pointcloud_t::ConstPtr cloud);

  bool WasVisual() const;
  bool WasStopped() const;

 private:
  void ConvertToPointCloud(const cv::Mat &rgb, const cv::Mat& depth,
      pointcloud_t::Ptr cloud);

  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

  mynteye::CameraIntrinsics cam_in_;
  float cam_factor_;
};

#endif  // MYNTEYE_SAMPLES_PC_VIEWER_H_ NOLINT
