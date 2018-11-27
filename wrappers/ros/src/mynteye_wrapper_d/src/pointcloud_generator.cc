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
#include "pointcloud_generator.h"

#include <utility>

MYNTEYE_USE_NAMESPACE

PointCloudGenerator::PointCloudGenerator(CameraIntrinsics in, Callback callback,
    double factor, std::int32_t frequency)
  : in_(std::move(in)),
    callback_(std::move(callback)),
    rate_(nullptr),
    running_(false),
    generating_(false),
    factor_(factor) {
  if (frequency > 0) {
    rate_.reset(new MYNTEYE_NAMESPACE::Rate(frequency));
  }
  Start();
}

PointCloudGenerator::~PointCloudGenerator() {
  Stop();
}

bool PointCloudGenerator::Push(const cv::Mat& color, const cv::Mat& depth,
    ros::Time stamp) {
  if (!running_) {
    throw new std::runtime_error("Start first!");
  }
  if (generating_) return false;
  {
    std::lock_guard<std::mutex> _(mutex_);
    if (generating_) return false;
    generating_ = true;
    color_ = color.clone();
    depth_ = depth.clone();
    stamp_ = stamp;
  }
  condition_.notify_one();
  return true;
}

void PointCloudGenerator::Start() {
  if (running_) return;
  {
    std::lock_guard<std::mutex> _(mutex_);
    if (running_) return;
    running_ = true;
  }
  thread_ = std::thread(&PointCloudGenerator::Run, this);
}

void PointCloudGenerator::Stop() {
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

void PointCloudGenerator::Run() {
  while (running_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return generating_; });
      if (!running_) break;
    }

    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = stamp_;
    msg.width = depth_.cols;
    msg.height = depth_.rows;
    msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg);

    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    sensor_msgs::PointCloud2Iterator<uchar> iter_r(msg, "r");
    sensor_msgs::PointCloud2Iterator<uchar> iter_g(msg, "g");
    sensor_msgs::PointCloud2Iterator<uchar> iter_b(msg, "b");

    for (int m = 0; m < depth_.rows; m++) {
      for (int n = 0; n < depth_.cols; n++) {
        // get depth value at (m, n)
        ushort d = depth_.ptr<ushort>(m)[n];
        // when d is equal 0 or 4096 means no depth
        if (d == 0 || d == 4096)
          continue;

        *iter_z = d / factor_;
        *iter_x = (n - in_.cx) * *iter_z / in_.fx;
        *iter_y = (m - in_.cy) * *iter_z / in_.fy;

        *iter_r = color_.ptr<uchar>(m)[n * 3 + 2];
        *iter_g = color_.ptr<uchar>(m)[n * 3 + 1];
        *iter_b = color_.ptr<uchar>(m)[n * 3];

        ++iter_x; ++iter_y; ++iter_z;  // NOLINT
        ++iter_r; ++iter_g; ++iter_b;  // NOLINT
      }
    }

    if (callback_) {
      callback_(std::move(msg));
    }

    if (rate_) {
      rate_->Sleep();
    }
    {
      std::lock_guard<std::mutex> _(mutex_);
      generating_ = false;
    }
  }
}
