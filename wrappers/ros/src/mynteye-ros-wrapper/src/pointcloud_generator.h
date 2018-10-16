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
