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

#include <mynteye_wrapper_d/Temp.h>

#include "mynteye/camera.h"
#include "mynteye/utils.h"

#include "pointcloud_generator.h" // NOLINT

MYNTEYE_BEGIN_NAMESPACE

namespace enc = sensor_msgs::image_encodings;

class MYNTEYEWrapperNodelet : public nodelet::Nodelet {
 public:
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns;

  image_transport::CameraPublisher pub_left_mono;
  image_transport::CameraPublisher pub_left_color;
  image_transport::CameraPublisher pub_right_mono;
  image_transport::CameraPublisher pub_right_color;
  image_transport::CameraPublisher pub_depth;
  ros::Publisher pub_points;
  ros::Publisher pub_imu;
  ros::Publisher pub_temp;

  sensor_msgs::CameraInfoPtr left_info_ptr;
  sensor_msgs::CameraInfoPtr right_info_ptr;

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
  std::string imu_frame_id;
  std::string temp_frame_id;

  // MYNTEYE objects
  OpenParams params;
  std::unique_ptr<Camera> mynteye;

  // Others

  std::unique_ptr<PointCloudGenerator> pointcloud_generator;

  std::shared_ptr<ImuData> imu_accel;
  std::shared_ptr<ImuData> imu_gyro;

  cv::Mat points_color;
  cv::Mat points_depth;

  typedef struct SubResult {
    bool left_mono;
    bool left_color;
    bool right_mono;
    bool right_color;
    bool depth;
    bool points;
    bool imu;
    bool temp;
    bool left;
    bool right;
  } sub_result_t;

  sub_result_t sub_result;

  MYNTEYEWrapperNodelet() {
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
    int color_mode = 0;
    int depth_mode = 0;
    int stream_mode = 0;
    int device_mode = 2;
    int color_stream_format = 0;
    int depth_stream_format = 0;
    bool state_ae = true;
    bool state_awb = true;
    int ir_intensity = 0;
    int proc_mode = 0;
    nh_ns.getParam("dev_index", dev_index);
    nh_ns.getParam("framerate", framerate);
    nh_ns.getParam("color_mode", color_mode);
    nh_ns.getParam("depth_mode", depth_mode);
    nh_ns.getParam("stream_mode", stream_mode);
    nh_ns.getParam("device_mode", device_mode);
    nh_ns.getParam("color_stream_format", color_stream_format);
    nh_ns.getParam("depth_stream_format", depth_stream_format);
    nh_ns.getParam("state_ae", state_ae);
    nh_ns.getParam("state_awb", state_awb);
    nh_ns.getParam("ir_intensity", ir_intensity);
    nh_ns.getParam("proc_mode", proc_mode);

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
    imu_frame_id = "mynteye_imu_frame";
    temp_frame_id = "mynteye_temp_frame";
    nh_ns.getParam("base_frame_id", base_frame_id);
    nh_ns.getParam("left_mono_frame", left_mono_frame_id);
    nh_ns.getParam("left_color_frame", left_color_frame_id);
    nh_ns.getParam("right_mono_frame", right_mono_frame_id);
    nh_ns.getParam("right_color_frame", right_color_frame_id);
    nh_ns.getParam("depth_frame", depth_frame_id);
    nh_ns.getParam("points_frame", points_frame_id);
    nh_ns.getParam("imu_frame", imu_frame_id);
    nh_ns.getParam("temp_frame", temp_frame_id);
    NODELET_INFO_STREAM("base_frame: " << base_frame_id);
    NODELET_INFO_STREAM("left_mono_frame: " << left_mono_frame_id);
    NODELET_INFO_STREAM("left_color_frame: " << left_color_frame_id);
    NODELET_INFO_STREAM("right_mono_frame: " << right_mono_frame_id);
    NODELET_INFO_STREAM("right_color_frame: " << right_color_frame_id);
    NODELET_INFO_STREAM("depth_frame: " << depth_frame_id);
    NODELET_INFO_STREAM("points_frame: " << points_frame_id);
    NODELET_INFO_STREAM("imu_frame: " << imu_frame_id);
    NODELET_INFO_STREAM("temp_frame: " << temp_frame_id);

    std::string left_mono_topic = "mynteye/left/image_mono";
    std::string left_color_topic = "mynteye/left/image_color";
    std::string right_mono_topic = "mynteye/right/image_mono";
    std::string right_color_topic = "mynteye/right/image_color";
    std::string depth_topic = "mynteye/depth";
    std::string points_topic = "mynteye/points";
    std::string imu_topic = "mynteye/imu";
    std::string temp_topic = "mynteye/temp";
    nh_ns.getParam("left_mono_topic", left_mono_topic);
    nh_ns.getParam("left_color_topic", left_color_topic);
    nh_ns.getParam("right_mono_topic", right_mono_topic);
    nh_ns.getParam("right_color_topic", right_color_topic);
    nh_ns.getParam("depth_topic", depth_topic);
    nh_ns.getParam("points_topic", points_topic);
    nh_ns.getParam("imu_topic", imu_topic);
    nh_ns.getParam("temp_topic", temp_topic);

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
    params.color_mode = static_cast<ColorMode>(color_mode);
    params.depth_mode = static_cast<DepthMode>(depth_mode);
    params.stream_mode = static_cast<StreamMode>(stream_mode);
    params.device_mode = static_cast<DeviceMode>(device_mode);
    params.color_stream_format =
        static_cast<StreamFormat>(color_stream_format);
    params.depth_stream_format =
        static_cast<StreamFormat>(depth_stream_format);
    params.state_ae = state_ae;
    params.state_awb = state_awb;
    params.ir_intensity = ir_intensity;

    mynteye->EnableProcessMode(proc_mode);

    // Image publishers

    image_transport::ImageTransport it_mynteye(nh);
    // left
    pub_left_mono = it_mynteye.advertiseCamera(left_mono_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_mono_topic);
    pub_left_color = it_mynteye.advertiseCamera(left_color_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_color_topic);
    // right
    pub_right_mono = it_mynteye.advertiseCamera(right_mono_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_mono_topic);
    pub_right_color = it_mynteye.advertiseCamera(right_color_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_color_topic);
    // depth
    pub_depth = it_mynteye.advertiseCamera(depth_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << depth_topic);
    // points
    pub_points = nh.advertise<sensor_msgs::PointCloud2>(points_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << points_topic);
    // imu
    pub_imu = nh.advertise<sensor_msgs::Imu>(imu_topic, 100);
    NODELET_INFO_STREAM("Advertized on topic " << imu_topic);
    // temp
    pub_temp = nh.advertise<mynteye_wrapper_d::Temp>(temp_topic, 100);
    NODELET_INFO_STREAM("Advertized on topic " << temp_topic);

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
    bool imu_sub = pub_imu.getNumSubscribers() > 0;
    bool temp_sub = pub_temp.getNumSubscribers() > 0;

    bool left_sub = left_mono_sub || left_color_sub;
    bool right_sub = right_mono_sub || right_color_sub;

    if (left_sub || right_sub || depth_sub || points_sub) {
      if (mynteye->IsImageInfoSupported()) {
        mynteye->EnableImageInfo(true);
      }
      if (left_sub || points_sub) {
        mynteye->EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
      }
      if (right_sub && util::is_right_color_supported(params.stream_mode)) {
        mynteye->EnableStreamData(ImageType::IMAGE_RIGHT_COLOR);
      }
      if (depth_sub || points_sub) {
        mynteye->EnableStreamData(ImageType::IMAGE_DEPTH);
      }
    } else {
      mynteye->DisableImageInfo();
      mynteye->DisableStreamData(ImageType::IMAGE_ALL);
    }

    if (imu_sub || temp_sub) {
      if (mynteye->IsMotionDatasSupported()) {
        mynteye->EnableMotionDatas(0);
      }
    } else {
      mynteye->DisableMotionDatas();
    }

    sub_result = {
      left_mono_sub, left_color_sub, right_mono_sub, right_color_sub,
      depth_sub, points_sub, imu_sub, temp_sub, left_sub, right_sub,
    };
  }

  void openDevice() {
    // if (!mynteye) {
    //   NODELET_ERROR_STREAM("Init firstly");
    // }
    if (mynteye->IsOpened()) return;

    // Set stream data callbacks
    std::vector<ImageType> types{
      ImageType::IMAGE_LEFT_COLOR,
      ImageType::IMAGE_RIGHT_COLOR,
      ImageType::IMAGE_DEPTH,
    };
    for (auto&& type : types) {
      mynteye->SetStreamCallback(type, [this](const StreamData& data) {
        switch (data.img->type()) {
          case ImageType::IMAGE_LEFT_COLOR: {
            if (sub_result.left || sub_result.points) {
              publishLeft(data, sub_result.left_color, sub_result.left_mono);
            }
          } break;
          case ImageType::IMAGE_RIGHT_COLOR: {
            if (sub_result.right) {
              publishRight(data, sub_result.right_color, sub_result.right_mono);
            }
          } break;
          case ImageType::IMAGE_DEPTH: {
            if (sub_result.depth || sub_result.points) {
              publishDepth(data);
            }
          } break;
        }
      });
    }

    // Set motion data callback
    mynteye->SetMotionCallback([this](const MotionData& data) {
      if (data.imu && (sub_result.imu || sub_result.temp)) {
        ros::Time stamp = hardTimeToSoftTime(data.imu->timestamp);
        if (data.imu->flag == MYNTEYE_IMU_ACCEL) {
          imu_accel = data.imu;
          publishImu(stamp, sub_result.imu, sub_result.temp);
        } else if (data.imu->flag == MYNTEYE_IMU_GYRO) {
          imu_gyro = data.imu;
          publishImu(stamp, sub_result.imu, sub_result.temp);
        }
      }
    });

    mynteye->Open(params);
    if (!mynteye->IsOpened()) {
      NODELET_ERROR_STREAM("Open camera failed");
      return;
    }
    NODELET_INFO_STREAM("Open camera success");

    // camera infos
    bool in_ok;
    auto&& in = mynteye->GetStreamIntrinsics(params.stream_mode, &in_ok);
    if (in_ok) {
      NODELET_INFO_STREAM("Camera info is created");
      left_info_ptr = createCameraInfo(in.left);
      right_info_ptr = createCameraInfo(in.right);
    } else {
      NODELET_WARN_STREAM("Camera info is null");
      left_info_ptr = nullptr;
      right_info_ptr = nullptr;
    }

    // pointcloud generator
    pointcloud_generator.reset(new PointCloudGenerator(
      in_ok ? in.left : getDefaultCameraIntrinsics(params.stream_mode),
      [this](sensor_msgs::PointCloud2 msg) {
        // msg.header.seq = 0;
        // msg.header.stamp = ros::Time::now();
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
    publishColor(data, left_info_ptr,
        pub_left_color, color_sub, left_color_frame_id,
        pub_left_mono, mono_sub, left_mono_frame_id, true);
  }

  void publishRight(const StreamData& data, bool color_sub, bool mono_sub) {
    publishColor(data, right_info_ptr,
        pub_right_color, color_sub, right_color_frame_id,
        pub_right_mono, mono_sub, right_mono_frame_id, false);
  }

  void publishColor(const StreamData& data,
      const sensor_msgs::CameraInfoPtr& info,
      const image_transport::CameraPublisher& pub_color, bool color_sub,
      const std::string color_frame_id,
      const image_transport::CameraPublisher& pub_mono, bool mono_sub,
      const std::string mono_frame_id, bool is_left) {
    auto timestamp = data.img_info
        ? hardTimeToSoftTime(data.img_info->timestamp)
        : ros::Time().now();
    auto&& mat = data.img->To(ImageFormat::COLOR_RGB)->ToMat();

    if (color_sub) {
      std_msgs::Header header;
      // header.seq = 0;
      header.stamp = timestamp;
      header.frame_id = color_frame_id;

      auto&& msg = cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg();
      if (info) info->header.stamp = msg->header.stamp;
      pub_color.publish(msg, info);
    }
    if (mono_sub) {
      std_msgs::Header header;
      // header.seq = 0;
      header.stamp = timestamp;
      header.frame_id = mono_frame_id;

      cv::Mat dst;
      cv::cvtColor(mat, dst, CV_RGB2GRAY);
      auto&& msg = cv_bridge::CvImage(header, enc::MONO8, dst).toImageMsg();
      if (info) info->header.stamp = msg->header.stamp;
      pub_mono.publish(msg, info);
    }

    if (is_left && sub_result.points) {
      points_color = mat;
      publishPoints(timestamp);
    }
  }

  void publishDepth(const StreamData& data) {
    std_msgs::Header header;
    // header.seq = 0;
    header.stamp = data.img_info
        ? hardTimeToSoftTime(data.img_info->timestamp)
        : ros::Time().now();
    header.frame_id = depth_frame_id;

    auto&& info = left_info_ptr;
    if (info) info->header.stamp = header.stamp;
    if (params.depth_mode == DepthMode::DEPTH_RAW) {
      auto&& mat = data.img->To(ImageFormat::DEPTH_RAW)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::MONO16, mat).toImageMsg(), info);
      /*{
        sensor_msgs::ImagePtr depth_msg =
            boost::make_shared<sensor_msgs::Image>();
        depth_msg->header = header;

        depth_msg->width = mat.cols;
        depth_msg->height = mat.rows;

        int num = 1;  // for endian detection
        depth_msg->is_bigendian = !(*(char*)&num == 1);  // NOLINT

        depth_msg->encoding = enc::MONO16;
        depth_msg->step = depth_msg->width * sizeof(std::uint16_t);

        std::size_t size = depth_msg->step * depth_msg->height;
        depth_msg->data.resize(size);

        std::uint16_t* data = reinterpret_cast<std::uint16_t*>(
            &depth_msg->data[0]);
        int data_size = depth_msg->width * depth_msg->height;

        ushort* depth_data_ptr = mat.ptr<ushort>();
        for (int i = 0; i < data_size; i++) {
          *(data++) = *(depth_data_ptr++);
        }

        pub_depth.publish(depth_msg, info);
      }*/

      if (sub_result.points) {
        points_depth = mat;
        publishPoints(header.stamp);
      }
    } else if (params.depth_mode == DepthMode::DEPTH_GRAY) {
      auto&& mat = data.img->To(ImageFormat::DEPTH_GRAY_24)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg(), info);
    } else if (params.depth_mode == DepthMode::DEPTH_COLORFUL) {
      auto&& mat = data.img->To(ImageFormat::DEPTH_RGB)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg(), info);
    } else {
      NODELET_ERROR_STREAM("Depth mode unsupported");
    }
  }

  void publishPoints(ros::Time stamp) {
    // NODELET_INFO_STREAM("publishPoints ..");
    if (points_color.empty() || points_depth.empty()) {
      // NODELET_INFO_STREAM("publishPoints skipped ..");
      return;
    }
    pointcloud_generator->Push(points_color, points_depth, stamp);
    points_color.release();
    points_depth.release();
  }

  void publishImu(ros::Time stamp, bool imu_sub, bool temp_sub) {
    if (imu_accel == nullptr || imu_gyro == nullptr) {
      return;
    }

    if (imu_sub) {
      sensor_msgs::Imu msg;

      // msg.header.seq = seq;
      msg.header.stamp = stamp;
      msg.header.frame_id = imu_frame_id;

      // acceleration should be in m/s^2 (not in g's)
      msg.linear_acceleration.x = imu_accel->accel[0] * gravity;
      msg.linear_acceleration.y = imu_accel->accel[1] * gravity;
      msg.linear_acceleration.z = imu_accel->accel[2] * gravity;

      msg.linear_acceleration_covariance[0] = 0;
      msg.linear_acceleration_covariance[1] = 0;
      msg.linear_acceleration_covariance[2] = 0;

      msg.linear_acceleration_covariance[3] = 0;
      msg.linear_acceleration_covariance[4] = 0;
      msg.linear_acceleration_covariance[5] = 0;

      msg.linear_acceleration_covariance[6] = 0;
      msg.linear_acceleration_covariance[7] = 0;
      msg.linear_acceleration_covariance[8] = 0;

      // velocity should be in rad/sec
      msg.angular_velocity.x = imu_gyro->gyro[0] * M_PI / 180;
      msg.angular_velocity.y = imu_gyro->gyro[1] * M_PI / 180;
      msg.angular_velocity.z = imu_gyro->gyro[2] * M_PI / 180;

      msg.angular_velocity_covariance[0] = 0;
      msg.angular_velocity_covariance[1] = 0;
      msg.angular_velocity_covariance[2] = 0;

      msg.angular_velocity_covariance[3] = 0;
      msg.angular_velocity_covariance[4] = 0;
      msg.angular_velocity_covariance[5] = 0;

      msg.angular_velocity_covariance[6] = 0;
      msg.angular_velocity_covariance[7] = 0;
      msg.angular_velocity_covariance[8] = 0;

      pub_imu.publish(msg);
    }

    if (temp_sub) {
      publishTemp(imu_accel->temperature, stamp);
    }

    imu_accel = nullptr;
    imu_gyro = nullptr;

    ros::Duration(0.001).sleep();
  }

  void publishTemp(float temperature, ros::Time stamp) {
    mynteye_wrapper_d::Temp msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = temp_frame_id;
    msg.data = temperature;
    pub_temp.publish(msg);
  }

  sensor_msgs::CameraInfoPtr createCameraInfo(const CameraIntrinsics& in) {
    // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html
    sensor_msgs::CameraInfo *camera_info = new sensor_msgs::CameraInfo();
    auto camera_info_ptr = sensor_msgs::CameraInfoPtr(camera_info);

    // camera_info->header.frame_id = color_frame_id;
    camera_info->width = in.width;
    camera_info->height = in.height;

    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    camera_info->K.at(0) = in.fx;
    camera_info->K.at(2) = in.cx;
    camera_info->K.at(4) = in.fy;
    camera_info->K.at(5) = in.cy;
    camera_info->K.at(8) = 1;

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    // for (int i = 0; i < 12; i++) {
    //     camera_info->P.at(i) = in.NewCamMat1[i];
    // }

    camera_info->distortion_model = "plumb_bob";

    // D of plumb_bob: (k1, k2, t1, t2, k3)
    for (int i = 0; i < 5; i++) {
      camera_info->D.push_back(in.coeffs[i]);
    }

    // R to identity matrix
    camera_info->R.at(0) = 1.0;
    camera_info->R.at(1) = 0.0;
    camera_info->R.at(2) = 0.0;
    camera_info->R.at(3) = 0.0;
    camera_info->R.at(4) = 1.0;
    camera_info->R.at(5) = 0.0;
    camera_info->R.at(6) = 0.0;
    camera_info->R.at(7) = 0.0;
    camera_info->R.at(8) = 1.0;

    return camera_info_ptr;
  }

  ros::Time hardTimeToSoftTime(double _hard_time) {
    static bool isInited = false;
    static double soft_time_begin(0), hard_time_begin(0);

    if (false == isInited) {
      soft_time_begin = ros::Time::now().toSec();
      hard_time_begin = _hard_time;
      isInited = true;
    }

    return ros::Time(
        soft_time_begin + (_hard_time - hard_time_begin) * 0.00001f);
  }

  CameraIntrinsics getDefaultCameraIntrinsics(const StreamMode& mode) {
    // {w, h, fx, fy, cx, cy, coeffs[5]{k1,k2,p1,p2,k3}}
    switch (mode) {
      case StreamMode::STREAM_640x480:
        return {640, 480, 979.8, 942.8, 682.3 / 2, 254.9, {0, 0, 0, 0, 0}};
      case StreamMode::STREAM_1280x480:
        return {640, 480, 979.8, 942.8, 682.3, 254.9, {0, 0, 0, 0, 0}};
      case StreamMode::STREAM_1280x720:
        return {640, 480, 979.8, 942.8, 682.3, 254.9 * 2, {0, 0, 0, 0, 0}};
      case StreamMode::STREAM_2560x720:
        return {640, 480, 979.8, 942.8, 682.3 * 2, 254.9 * 2, {0, 0, 0, 0, 0}};
      default:
        return {640, 480, 979.8, 942.8, 682.3, 254.9, {0, 0, 0, 0, 0}};
    }
  }
};

MYNTEYE_END_NAMESPACE

#include <pluginlib/class_list_macros.h> // NOLINT
PLUGINLIB_EXPORT_CLASS(MYNTEYE_NAMESPACE::MYNTEYEWrapperNodelet, nodelet::Nodelet);
