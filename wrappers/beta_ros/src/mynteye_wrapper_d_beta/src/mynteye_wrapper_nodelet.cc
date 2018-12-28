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
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <unistd.h>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include <mynteye_wrapper_d_beta/Temp.h> // NOLINT

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"

#include "pointcloud_generator.h" // NOLINT

MYNTEYE_BEGIN_NAMESPACE

namespace {

void matrix_3x1(const double (*src1)[3], const double (*src2)[1],
    double (*dst)[1]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 1; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

void matrix_3x3(const double (*src1)[3], const double (*src2)[3],
    double (*dst)[3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

}  // namespace

namespace enc = sensor_msgs::image_encodings;

class MYNTEYEWrapperNodelet : public nodelet::Nodelet {
 public:
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns;

  pthread_mutex_t mutex_sub_result;

  image_transport::Publisher pub_left_mono;
  image_transport::Publisher pub_left_color;
  image_transport::Publisher pub_right_mono;
  image_transport::Publisher pub_right_color;
  image_transport::Publisher pub_depth;

  ros::Publisher pub_points;

  // Launch params

  std::int32_t points_frequency;
  double points_factor;
  int gravity;

  std::string base_frame_id;
  std::string left_mono_frame_id;
  std::string left_color_frame_id;
  std::string right_mono_frame_id;
  std::string right_color_frame_id;
  std::string depth_frame_id;
  std::string points_frame_id;

  // MYNTEYE objects
  OpenParams params;
  std::unique_ptr<Camera> mynteye;

  // Others

  std::unique_ptr<PointCloudGenerator> pointcloud_generator;

  cv::Mat points_color;
  cv::Mat points_depth;

  typedef struct SubResult {
    bool left_mono;
    bool left_color;
    bool right_mono;
    bool right_color;
    bool depth;
    bool points;
    bool left;
    bool right;
  } sub_result_t;

  sub_result_t sub_result;

  MYNTEYEWrapperNodelet() {
    pthread_mutex_init(&mutex_sub_result, nullptr);
  }

  ~MYNTEYEWrapperNodelet() {
    closeDevice();
    mynteye.reset(nullptr);
  }

  void onInit() override {
    std::string dashes(30, '-');

    nh = getMTNodeHandle();
    nh_ns = getMTPrivateNodeHandle();

    // Launch params
    int dev_index = 0;
    int framerate = 10;
    int dev_mode = 2;
    int color_mode = 0;
    int depth_mode = 0;
    int stream_mode = 0;
    int color_stream_format = 0;
    int depth_stream_format = 0;
    bool state_ae = true;
    bool state_awb = true;
    int ir_intensity = 0;
    bool ir_depth_only = true;
    nh_ns.getParam("dev_index", dev_index);
    nh_ns.getParam("framerate", framerate);
    nh_ns.getParam("dev_mode", dev_mode);
    nh_ns.getParam("color_mode", color_mode);
    nh_ns.getParam("depth_mode", depth_mode);
    nh_ns.getParam("stream_mode", stream_mode);
    nh_ns.getParam("color_stream_format", color_stream_format);
    nh_ns.getParam("depth_stream_format", depth_stream_format);
    nh_ns.getParam("state_ae", state_ae);
    nh_ns.getParam("state_awb", state_awb);
    nh_ns.getParam("ir_intensity", ir_intensity);
    nh_ns.getParam("ir_depth_only", ir_depth_only);

    points_frequency = DEFAULT_POINTS_FREQUENCE;
    points_factor = DEFAULT_POINTS_FACTOR;
    gravity = 9.8;
    nh_ns.getParam("points_frequency", points_frequency);
    nh_ns.getParam("points_factor", points_factor);
    nh_ns.getParam("gravity", gravity);

    base_frame_id = "mynteye_link";
    left_mono_frame_id = "mynteye_left_mono_frame";
    left_color_frame_id = "mynteye_left_color_frame";
    right_mono_frame_id = "mynteye_right_mono_frame";
    right_color_frame_id = "mynteye_right_color_frame";
    depth_frame_id = "mynteye_depth_frame";
    points_frame_id = "mynteye_points_frame";
    nh_ns.getParam("base_frame_id", base_frame_id);
    nh_ns.getParam("left_mono_frame", left_mono_frame_id);
    nh_ns.getParam("left_color_frame", left_color_frame_id);
    nh_ns.getParam("right_mono_frame", right_mono_frame_id);
    nh_ns.getParam("right_color_frame", right_color_frame_id);
    nh_ns.getParam("depth_frame", depth_frame_id);
    nh_ns.getParam("points_frame", points_frame_id);
    NODELET_INFO_STREAM("base_frame: " << base_frame_id);
    NODELET_INFO_STREAM("left_mono_frame: " << left_mono_frame_id);
    NODELET_INFO_STREAM("left_color_frame: " << left_color_frame_id);
    NODELET_INFO_STREAM("right_mono_frame: " << right_mono_frame_id);
    NODELET_INFO_STREAM("right_color_frame: " << right_color_frame_id);
    NODELET_INFO_STREAM("depth_frame: " << depth_frame_id);
    NODELET_INFO_STREAM("points_frame: " << points_frame_id);

    std::string left_mono_topic = "mynteye/left/image_mono";
    std::string left_color_topic = "mynteye/left/image_color";
    std::string right_mono_topic = "mynteye/right/image_mono";
    std::string right_color_topic = "mynteye/right/image_color";
    std::string depth_topic = "mynteye/depth";
    std::string points_topic = "mynteye/points";
    nh_ns.getParam("left_mono_topic", left_mono_topic);
    nh_ns.getParam("left_color_topic", left_color_topic);
    nh_ns.getParam("right_mono_topic", right_mono_topic);
    nh_ns.getParam("right_color_topic", right_color_topic);
    nh_ns.getParam("depth_topic", depth_topic);
    nh_ns.getParam("points_topic", points_topic);

    // MYNTEYE objects
    mynteye.reset(new Camera);
    {
      std::vector<DeviceInfo> dev_infos = mynteye->GetDeviceInfos();
      size_t n = dev_infos.size();
      if (n <= 0 || dev_index < 0 || dev_index >= n) {
        NODELET_ERROR_STREAM("Device not found, index: " << dev_index);
        return;
      }

      NODELET_INFO_STREAM(dashes);
      NODELET_INFO_STREAM("Device Information");
      NODELET_INFO_STREAM(dashes);
      for (auto &&info : dev_infos) {
        NODELET_INFO_STREAM(info.index << " | " << info);
      }
      NODELET_INFO_STREAM(dashes);

      params.dev_index = dev_index;
    }
    {
      std::vector<StreamInfo> color_infos;
      std::vector<StreamInfo> depth_infos;
      mynteye->GetStreamInfos(dev_index, &color_infos, &depth_infos);

      NODELET_INFO_STREAM("Color Stream Information");
      NODELET_INFO_STREAM(dashes);
      for (auto &&info : color_infos) {
        NODELET_INFO_STREAM(info.index << " | " << info);
      }
      NODELET_INFO_STREAM(dashes);

      NODELET_INFO_STREAM("Depth Stream Information");
      NODELET_INFO_STREAM(dashes);
      for (auto &&info : depth_infos) {
        NODELET_INFO_STREAM(info.index << " | " << info);
      }
      NODELET_INFO_STREAM(dashes);
    }
    params.framerate = framerate;
    params.dev_mode = static_cast<DeviceMode>(dev_mode);
    params.color_mode = static_cast<ColorMode>(color_mode);
    params.depth_mode = static_cast<DepthMode>(depth_mode);
    params.stream_mode = static_cast<StreamMode>(stream_mode);
    params.color_stream_format =
        static_cast<StreamFormat>(color_stream_format);
    params.depth_stream_format =
        static_cast<StreamFormat>(depth_stream_format);
    params.state_ae = state_ae;
    params.state_awb = state_awb;
    params.ir_intensity = ir_intensity;
    params.ir_depth_only = ir_depth_only;

    // Image publishers

    image_transport::ImageTransport it_mynteye(nh);
    // left
    pub_left_mono = it_mynteye.advertise(left_mono_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_mono_topic);
    pub_left_color = it_mynteye.advertise(left_color_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_color_topic);
    // right
    pub_right_mono = it_mynteye.advertise(right_mono_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_mono_topic);
    pub_right_color = it_mynteye.advertise(right_color_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_color_topic);
    // depth
    pub_depth = it_mynteye.advertise(depth_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << depth_topic);
    // points
    pub_points = nh.advertise<sensor_msgs::PointCloud2>(points_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << points_topic);

    // detect subscribers to enable/disable features
    detectSubscribers();
    // open device
    openDevice();

    // loop
    ros::Rate loop_rate(framerate);
    while (nh_ns.ok()) {
      detectSubscribers();
      loop_rate.sleep();
    }
  }

  void detectSubscribers() {
    bool left_mono_sub = pub_left_mono.getNumSubscribers() > 0;
    bool left_color_sub = pub_left_color.getNumSubscribers() > 0;
    bool right_mono_sub = pub_right_mono.getNumSubscribers() > 0;
    bool right_color_sub = pub_right_color.getNumSubscribers() > 0;
    bool depth_sub = pub_depth.getNumSubscribers() > 0;
    bool points_sub = pub_points.getNumSubscribers() > 0;

    bool left_sub = left_mono_sub || left_color_sub;
    bool right_sub = right_mono_sub || right_color_sub;

    pthread_mutex_lock(&mutex_sub_result);

    sub_result = {
      left_mono_sub, left_color_sub, right_mono_sub, right_color_sub,
      depth_sub, points_sub, left_sub, right_sub,
    };
    pthread_mutex_unlock(&mutex_sub_result);
  }

  void openDevice() {
    if (mynteye->IsOpened()) return;

    // Set stream data callbacks
    std::vector<ImageType> types{
      ImageType::IMAGE_LEFT_COLOR,
      ImageType::IMAGE_RIGHT_COLOR,
      ImageType::IMAGE_DEPTH,
    };
    for (auto&& type : types) {
      mynteye->SetStreamCallback(type, [this](const StreamData& data) {
        pthread_mutex_lock(&mutex_sub_result);
        switch (data.img->type()) {
          case ImageType::IMAGE_LEFT_COLOR: {
            if (sub_result.left || sub_result.points) {
              publishLeft(data, sub_result.left_color,
                  sub_result.left_mono);
            }
          } break;
          case ImageType::IMAGE_RIGHT_COLOR: {
            if (sub_result.right) {
              publishRight(data, sub_result.right_color,
                  sub_result.right_mono);
            }
          } break;
          case ImageType::IMAGE_DEPTH: {
            if (sub_result.depth || sub_result.points) {
              publishDepth(data);
            }
          } break;
        }
        pthread_mutex_unlock(&mutex_sub_result);
      });
    }

    mynteye->Open(params);
    if (!mynteye->IsOpened()) {
      NODELET_ERROR_STREAM("Open camera failed");
      return;
    }
    NODELET_INFO_STREAM("Open camera success");

    // pointcloud generator
    pointcloud_generator.reset(new PointCloudGenerator(
        getDefaultCameraIntrinsics(params.stream_mode),
        [this](sensor_msgs::PointCloud2 msg) {
          msg.header.frame_id = points_frame_id;
          pub_points.publish(msg);
        }, points_factor, points_frequency));
  }

  void closeDevice() {
    if (mynteye) {
      mynteye->Close();
    }
  }

  void publishLeft(const StreamData& data, bool color_sub, bool mono_sub) {
    publishColor(data, pub_left_color,
        color_sub, left_color_frame_id,
        pub_left_mono, mono_sub, left_mono_frame_id, true);
  }

  void publishRight(const StreamData& data, bool color_sub, bool mono_sub) {
    publishColor(data, pub_right_color,
        color_sub, right_color_frame_id,
        pub_right_mono, mono_sub, right_mono_frame_id, false);
  }

  void publishColor(const StreamData& data,
      const image_transport::Publisher& pub_color,
      bool color_sub, const std::string color_frame_id,
      const image_transport::Publisher& pub_mono,
      bool mono_sub, const std::string mono_frame_id, bool is_left) {
    auto timestamp = ros::Time().now();
    auto&& mat = data.img->To(ImageFormat::COLOR_RGB)->ToMat();

    if (color_sub) {
      std_msgs::Header header;
      header.stamp = timestamp;
      header.frame_id = color_frame_id;

      auto&& msg = cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg();
      pub_color.publish(msg);
    }
    if (mono_sub) {
      std_msgs::Header header;
      header.stamp = timestamp;
      header.frame_id = mono_frame_id;

      cv::Mat dst;
      cv::cvtColor(mat, dst, CV_RGB2GRAY);
      auto&& msg = cv_bridge::CvImage(header, enc::MONO8, dst).toImageMsg();
      pub_mono.publish(msg);
    }

    if (is_left && sub_result.points) {
      points_color = mat;
      publishPoints(timestamp);
    }
  }

  void publishDepth(const StreamData& data) {
    std_msgs::Header header;
    header.stamp = ros::Time().now();
    header.frame_id = depth_frame_id;

    if (params.depth_mode == DepthMode::DEPTH_RAW) {
      auto&& mat = data.img->To(ImageFormat::DEPTH_RAW)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::MONO16, mat).toImageMsg());
      if (sub_result.points) {
        points_depth = mat;
        publishPoints(header.stamp);
      }
    } else if (params.depth_mode == DepthMode::DEPTH_GRAY) {
      auto&& mat = data.img->To(ImageFormat::DEPTH_GRAY_24)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg());
    } else if (params.depth_mode == DepthMode::DEPTH_COLORFUL) {
      auto&& mat = data.img->To(ImageFormat::DEPTH_RGB)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg());
    } else {
      NODELET_ERROR_STREAM("Depth mode unsupported");
    }
  }

  void publishPoints(ros::Time stamp) {
    if (points_color.empty() || points_depth.empty()) {
      return;
    }
    pointcloud_generator->Push(points_color, points_depth, stamp);
    points_color.release();
    points_depth.release();
  }

  ros::Time hardTimeToSoftTime(std::uint64_t _hard_time) {
    static bool isInited = false;
    static uint64_t hard_time_begin(0);
    static uint32_t soft_time_begin(0);
    if (false == isInited) {
      soft_time_begin = ros::Time::now().toSec();
      hard_time_begin = _hard_time;
      isInited = true;
    }
    std::uint64_t time_ns_detal = (_hard_time - hard_time_begin);
    std::uint64_t time_ns_detal_s = time_ns_detal / 100000;
    std::uint64_t time_ns_detal_ns = time_ns_detal % 100000;
    return ros::Time(soft_time_begin + time_ns_detal_s,
                     time_ns_detal_ns * 10000);
  }

  CameraIntrinsics getDefaultCameraIntrinsics(const StreamMode& mode) {
    // {w, h, fx, fy, cx, cy, coeffs[5]{k1,k2,p1,p2,k3}}
    switch (mode) {
      case StreamMode::STREAM_640x480:
        return {640, 480, 979.8, 942.8, 682.3 / 2, 254.9, {0, 0, 0, 0, 0}};
      case StreamMode::STREAM_1280x480:
        return {640, 480, 979.8, 942.8, 682.3, 254.9, {0, 0, 0, 0, 0}};
      case StreamMode::STREAM_1280x720:
        return {1280, 720, 979.8, 942.8, 682.3, 254.9 * 2, {0, 0, 0, 0, 0}};
      case StreamMode::STREAM_2560x720:
        return {1280, 720, 979.8, 942.8, 682.3 * 2, 254.9 * 2, {0, 0, 0, 0, 0}};
      default:
        return {1280, 720, 979.8, 942.8, 682.3, 254.9 * 2, {0, 0, 0, 0, 0}};
    }
  }
};

MYNTEYE_END_NAMESPACE

#include <pluginlib/class_list_macros.h> // NOLINT
PLUGINLIB_EXPORT_CLASS(MYNTEYE_NAMESPACE::MYNTEYEWrapperNodelet, nodelet::Nodelet); // NOLINT
