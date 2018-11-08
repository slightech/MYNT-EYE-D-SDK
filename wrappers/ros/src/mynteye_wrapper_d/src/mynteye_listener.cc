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
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

class MYNTEYEListener {
 public:
  MYNTEYEListener() : it_(nh_), color_count_(0) {
    color_sub_ =  it_.subscribe("mynteye/color", 1,
        &MYNTEYEListener::colorCallback, this);
    depth_sub_ =  it_.subscribe("mynteye/depth", 1,
        &MYNTEYEListener::depthCallback, this);

    cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
  }

  ~MYNTEYEListener() {
    cv::destroyAllWindows();
  }

  void colorCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, enc::RGB8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
      return;
    }
    ++color_count_;
    // ROS_INFO_STREAM("color: " << color_count_);

    cv::imshow("color", cv_ptr->image);
    cv::waitKey(3);
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      if (enc::isColor(msg->encoding)) {
        cv_ptr = cv_bridge::toCvShare(msg, enc::RGB8);
      } else if (msg->encoding == enc::MONO16) {
        cv_ptr = cv_bridge::toCvShare(msg, enc::MONO16);
      } else {
        cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
      }
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
      return;
    }
    cv::imshow("depth", cv_ptr->image);
    cv::waitKey(3);
  }

  std::uint64_t colorCount() const {
    return color_count_;
  }

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber color_sub_;
  image_transport::Subscriber depth_sub_;

  std::uint64_t color_count_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mynteye_listener_d");

  MYNTEYEListener l;

  double time_beg = ros::Time::now().toSec();
  ros::spin();
  double time_end = ros::Time::now().toSec();

  double elapsed = time_end - time_beg;
  ROS_INFO_STREAM("time beg: " << std::fixed << time_beg << " s");
  ROS_INFO_STREAM("time end: " << std::fixed << time_end << " s");
  ROS_INFO_STREAM("time cost: " << elapsed << " s");
  ROS_INFO_STREAM("color count: " << l.colorCount() << ", "
      << (l.colorCount() / elapsed) << " fps");

  return 0;
}
