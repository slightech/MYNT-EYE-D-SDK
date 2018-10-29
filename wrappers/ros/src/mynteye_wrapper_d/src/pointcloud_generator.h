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
#ifndef MYNTEYE_WRAPPER_POINTCLOUD_GENERATOR_H_
#define MYNTEYE_WRAPPER_POINTCLOUD_GENERATOR_H_
#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <opencv2/core/core.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "mynteye/util/rate.h"

MYNTEYE_BEGIN_NAMESPACE

struct CameraIntrinsics {
  double factor;
  double cx;
  double cy;
  double fx;
  double fy;
};

class PointCloudGenerator {
 public:
  using Callback = std::function<void(sensor_msgs::PointCloud2)>;

  PointCloudGenerator(CameraIntrinsics in, Callback callback,
      std::int32_t frequency = 0);
  ~PointCloudGenerator();

  bool Push(cv::Mat color, cv::Mat depth, ros::Time stamp);

 private:
  void Start();
  void Stop();

  void Run();

  CameraIntrinsics in_;
  Callback callback_;

  std::unique_ptr<mynteye::Rate> rate_;

  std::mutex mutex_;
  std::condition_variable condition_;

  bool running_;
  std::thread thread_;

  cv::Mat color_;
  cv::Mat depth_;
  ros::Time stamp_;

  bool generating_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_WRAPPER_POINTCLOUD_GENERATOR_H_
