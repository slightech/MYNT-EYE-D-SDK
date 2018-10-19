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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye_wrapper/Temp.h"

#include "mynteye/camera.h"
#include "pointcloud_generator.h" // NOLINT

MYNTEYE_BEGIN_NAMESPACE

namespace enc = sensor_msgs::image_encodings;

class MYNTEYEWrapperNodelet : public nodelet::Nodelet {
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns;
  boost::shared_ptr<boost::thread> device_poll_thread;

  image_transport::CameraPublisher pub_color;
  image_transport::CameraPublisher pub_depth;
  image_transport::CameraPublisher pub_gray;
  ros::Publisher pub_points;
  ros::Publisher pub_imu;
  ros::Publisher pub_temp;

  // tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

  sensor_msgs::CameraInfoPtr camera_info_ptr_;

  // Launch params
  int dev_index;
  int framerate;
  int depth_mode;
  int stream_mode;
  int color_stream_format;
  int depth_stream_format;
  bool state_ae;
  bool state_awb;
  int ir_intensity;
  int gravity;

  std::string base_frame_id;
  std::string color_frame_id;
  std::string depth_frame_id;
  std::string points_frame_id;
  std::string imu_frame_id;
  std::string temp_frame_id;
  std::string gray_frame_id;

  // MYNTEYE objects
  mynteye::InitParams params;
  std::unique_ptr<mynteye::Camera> mynteye;

  std::unique_ptr<PointCloudGenerator> pointcloud_generator;

  std::shared_ptr<ImuData> imu_accel;
  std::shared_ptr<ImuData> imu_gyro;

  std::string dashes;

  ros::Time timeBeginPointOnRos;

  std::uint32_t timeBeginPointOnDevice;

  bool isImuTimeInited;

 public:
  MYNTEYEWrapperNodelet() : dashes(std::string(30, '-')) {
  }

  void publishColor(mynteye::Image::pointer img, ros::Time stamp,
      cv::Mat* mat, std::uint32_t seq) {
    if (pub_color.getNumSubscribers() == 0)
      return;
    std_msgs::Header header;
    // header.seq = 0;
    header.stamp = stamp;
    header.frame_id = color_frame_id;
    *mat = img->To(mynteye::ImageFormat::COLOR_RGB)->ToMat();

    auto &&msg =
        cv_bridge::CvImage(header, enc::RGB8, *mat).toImageMsg();
    auto &&info = getCameraInfo();
    info->header.stamp = msg->header.stamp;
    pub_color.publish(msg, info);
  }

  void publishGray(mynteye::Image::pointer img, ros::Time stamp,
      cv::Mat* mat, std::uint32_t seq) {
    if (pub_gray.getNumSubscribers() == 0)
      return;
    std_msgs::Header header;
    // header.seq = 0;
    header.stamp = stamp;
    header.frame_id = color_frame_id;
    *mat = img->To(mynteye::ImageFormat::IMAGE_GRAY_8)->ToMat();

    auto &&msg =
        cv_bridge::CvImage(header, enc::MONO8, *mat).toImageMsg();
    auto &&info = getCameraInfo();
    info->header.stamp = msg->header.stamp;
    pub_color.publish(msg, info);
  }

  sensor_msgs::CameraInfoPtr getCameraInfo() {
    // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html
    sensor_msgs::CameraInfo *camera_info = new sensor_msgs::CameraInfo();
    camera_info_ptr_ = sensor_msgs::CameraInfoPtr(camera_info);

    struct MYNTEYE_NAMESPACE::CameraCtrlRectLogData camera_ctrl_data;

    // <arg name="stream_1280x720"   default="0" />
    // <arg name="stream_2560x720"   default="1" />
    // GetHDCameraCtrlData();
    // <arg name="stream_1280x480"   default="2" />
    // <arg name="stream_640x480"    default="3" />
    // GetVGACameraCtrlData();

    if (stream_mode == 0 || stream_mode == 1) {
      camera_ctrl_data = mynteye->GetHDCameraCtrlData();
    } else if (stream_mode == 2 || stream_mode == 3) {
      camera_ctrl_data = mynteye->GetVGACameraCtrlData();
    }

    camera_info->header.frame_id = color_frame_id;
    camera_info->width = camera_ctrl_data.OutImgWidth;
    camera_info->height = camera_ctrl_data.OutImgHeight;

    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    camera_info->K.at(0) = camera_ctrl_data.CamMat1[0];
    camera_info->K.at(2) = camera_ctrl_data.CamMat1[2];
    camera_info->K.at(4) = camera_ctrl_data.CamMat1[4];
    camera_info->K.at(5) = camera_ctrl_data.CamMat1[5];
    camera_info->K.at(8) = 1;

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    for (int i = 0; i < 12; i++) {
        camera_info->P.at(i) = camera_ctrl_data.NewCamMat1[i];
    }

    camera_info->distortion_model = "plumb_bob";

    // D of plumb_bob: (k1, k2, t1, t2, k3)
    for (int i = 0; i < 5; i++) {
      camera_info->D.push_back(camera_ctrl_data.CamDist1[i]);
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

    return camera_info_ptr_;
  }

  void publishDepth(mynteye::Image::pointer img, ros::Time stamp,
      cv::Mat* mat) {
    std_msgs::Header header;
    // header.seq = 0;
    header.stamp = stamp;
    header.frame_id = depth_frame_id;

    auto &&info = getCameraInfo();
    if (depth_mode == 0) {  // DEPTH_RAW
      *mat = img->To(mynteye::ImageFormat::DEPTH_RAW)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::MONO16, *mat).toImageMsg(), info);
    } else if (depth_mode == 1) {  // DEPTH_GRAY
      *mat = img->To(mynteye::ImageFormat::DEPTH_GRAY_24)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::RGB8, *mat).toImageMsg(), info);
    } else if (depth_mode == 2) {  // DEPTH_COLORFUL
      *mat = img->To(mynteye::ImageFormat::DEPTH_RGB)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::RGB8, *mat).toImageMsg(), info);
    } else {
      NODELET_ERROR_STREAM("Depth mode unsupported");
      return;
    }
    // NODELET_INFO_STREAM("Publish depth");
  }

  void publishImu(ros::Time stamp, bool pub_temp) {
    if (imu_accel == nullptr || imu_gyro == nullptr) {
      return;
    }
    sensor_msgs::Imu msg;

    //msg.header.seq = seq;
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

    if (pub_temp) {
      publishTemp(imu_accel->temperature, stamp);
    }

    imu_accel = nullptr;
    imu_gyro = nullptr;
    // ros::Duration(0.001).sleep();
  }

  void publishTemp(float temperature, ros::Time stamp) {
    mynteye_wrapper::Temp msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = temp_frame_id;
    msg.data = temperature;
    pub_temp.publish(msg);
  }

  void device_poll() {
    mynteye->Open(params);
    if (!mynteye->IsOpened()) {
      NODELET_ERROR_STREAM("Open camera failed");
      return;
    }
    NODELET_INFO_STREAM("Open camera success");

    ros::Rate loop_rate(params.framerate);

    // Main loop
    cv::Mat color, gray, depth;
    while (nh_ns.ok()) {
      // Check for subscribers
      int color_SubNumber = pub_color.getNumSubscribers();
      int depth_SubNumber = pub_depth.getNumSubscribers();
      int points_SubNumber = pub_points.getNumSubscribers();
      int imu_SubNumber = pub_imu.getNumSubscribers();
      int temp_SubNumber = pub_temp.getNumSubscribers();
      int gray_SubNumber = pub_gray.getNumSubscribers();

      bool img_Sub = (color_SubNumber + depth_SubNumber + points_SubNumber) > 0;
      bool imu_Sub = (imu_SubNumber + temp_SubNumber) > 0;
      if (img_Sub || imu_Sub) {
        // publish points, the depth mode must be DEPTH_RAW.
        bool points_subscribed = (points_SubNumber > 0) && (depth_mode == 0);

        ros::Time t = ros::Time::now();

        bool color_ok = false;
        if (color_SubNumber > 0 || points_subscribed) {
          auto image_color = mynteye->RetrieveImage(
              mynteye::ImageType::IMAGE_COLOR);
          if (image_color.img) {
            color_ok = true;
            static std::size_t count = 0;
            ++count;
            publishColor(image_color.img, t, &color, count);
          }
        }

        bool gray_ok = false;
        if (gray_SubNumber > 0 || points_subscribed) {
          auto image_gray = mynteye->RetrieveImage(
              mynteye::ImageType::IMAGE_COLOR);
          if (image_gray.img) {
            gray_ok = true;
            static std::size_t count = 0;
            ++count;
            publishGray(image_gray.img, t, &gray, count);
          }
        }

        bool depth_ok = false;
        if (depth_SubNumber > 0 || points_subscribed) {
          auto image_depth = mynteye->RetrieveImage(
              mynteye::ImageType::IMAGE_DEPTH);
          if (image_depth.img) {
            depth_ok = true;
            publishDepth(image_depth.img, t, &depth);
          }
        }

        if (points_subscribed && color_ok && depth_ok) {
          pointcloud_generator->Push(color, depth, t);
        }

        if (imu_Sub) {
          // NODELET_INFO_STREAM("Retrieve motions");
          auto motion_datas = mynteye->RetrieveMotions();
          if (motion_datas.size() > 0) {
            for (auto data : motion_datas) {
              ros::Time tImg = timeBeginPointOnRos;
              if (isImuTimeInited) {
                tImg += ros::Duration(
                  (data.imu->timestamp - timeBeginPointOnDevice)/100000,
                  (data.imu->timestamp - timeBeginPointOnDevice)%100000*10000);
              } else {
                isImuTimeInited = true;
                timeBeginPointOnDevice = data.imu->timestamp;
                timeBeginPointOnRos = ros::Time::now();
              }
              if (data.imu) {
                if (data.imu->flag == 1) {  // accelerometer
                  imu_accel = data.imu;
                  publishImu(tImg, temp_SubNumber > 0);
                } else if (data.imu->flag == 2) {  // gyroscope
                  imu_gyro = data.imu;
                  publishImu(tImg, temp_SubNumber > 0);
                } else {
                  NODELET_WARN_STREAM("Imu type is unknown");
                }
              } else {
                NODELET_WARN_STREAM("Motion data is empty");
              }
            }
          }
        }
      }
      if (img_Sub) loop_rate.sleep();
    }

    mynteye.reset();
  }

  void onInit() {
    nh = getMTNodeHandle();
    nh_ns = getMTPrivateNodeHandle();

    // Launch params
    dev_index = 0;
    framerate = 10;
    depth_mode = 0;
    stream_mode = 0;
    color_stream_format = 0;
    depth_stream_format = 0;
    state_ae = true;
    state_awb = true;
    ir_intensity = 0;
    gravity = 9.8;
    isImuTimeInited = false;
    std::uint32_t timeBeginPointOnDevice = 0;

    nh_ns.getParam("dev_index", dev_index);
    nh_ns.getParam("framerate", framerate);
    nh_ns.getParam("depth_mode", depth_mode);
    nh_ns.getParam("stream_mode", stream_mode);
    nh_ns.getParam("color_stream_format", color_stream_format);
    nh_ns.getParam("depth_stream_format", depth_stream_format);
    nh_ns.getParam("state_ae", state_ae);
    nh_ns.getParam("state_awb", state_awb);
    nh_ns.getParam("ir_intensity", ir_intensity);
    nh_ns.getParam("gravity", gravity);

    base_frame_id = "mynteye_link";
    color_frame_id = "mynteye_color_frame";
    depth_frame_id = "mynteye_depth_frame";
    points_frame_id = "mynteye_points_frame";
    gray_frame_id = "mynteye_gray_frame";
    imu_frame_id = "mynteye_imu_frame";
    temp_frame_id = "mynteye_temp_frame";
    nh_ns.getParam("base_frame_id", base_frame_id);
    nh_ns.getParam("color_frame", color_frame_id);
    nh_ns.getParam("gray_frame", gray_frame_id);
    nh_ns.getParam("depth_frame", depth_frame_id);
    nh_ns.getParam("points_frame", points_frame_id);
    nh_ns.getParam("imu_frame", imu_frame_id);
    nh_ns.getParam("temp_frame", temp_frame_id);
    NODELET_INFO_STREAM("base_frame: " << base_frame_id);
    NODELET_INFO_STREAM("color_frame: " << color_frame_id);
    NODELET_INFO_STREAM("gray_frame: " << gray_frame_id);
    NODELET_INFO_STREAM("depth_frame: " << depth_frame_id);
    NODELET_INFO_STREAM("points_frame: " << points_frame_id);
    NODELET_INFO_STREAM("imu_frame: " << imu_frame_id);
    NODELET_INFO_STREAM("temp_frame: " << temp_frame_id);

    std::string color_topic = "mynteye/color";
    std::string depth_topic = "mynteye/depth";
    std::string points_topic = "mynteye/points";
    std::string imu_topic = "mynteye/imu";
    std::string temp_topic = "mynteye/temp";
    std::string gray_topic = "mynteye/gray";
    nh_ns.getParam("color_topic", color_topic);
    nh_ns.getParam("depth_topic", depth_topic);
    nh_ns.getParam("points_topic", points_topic);
    nh_ns.getParam("imu_topic", imu_topic);
    nh_ns.getParam("temp_topic", temp_topic);

    // MYNTEYE objects
    mynteye.reset(new mynteye::Camera);
    {
      std::vector<mynteye::DeviceInfo> dev_infos = mynteye->GetDevices();
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
      std::vector<mynteye::StreamInfo> color_infos;
      std::vector<mynteye::StreamInfo> depth_infos;
      mynteye->GetResolutions(dev_index, &color_infos, &depth_infos);

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
    params.depth_mode = static_cast<mynteye::DepthMode>(depth_mode);
    params.stream_mode = static_cast<mynteye::StreamMode>(stream_mode);
    params.color_stream_format =
        static_cast<mynteye::StreamFormat>(color_stream_format);
    params.depth_stream_format =
        static_cast<mynteye::StreamFormat>(depth_stream_format);
    params.state_ae = state_ae;
    params.state_awb = state_awb;
    params.ir_intensity = ir_intensity;

    // Image publishers
    image_transport::ImageTransport it_mynteye(nh);
    pub_color = it_mynteye.advertiseCamera(color_topic, 1);  // color
    NODELET_INFO_STREAM("Advertized on topic " << color_topic);
    pub_gray = it_mynteye.advertiseCamera(gray_topic, 1);  // gray
    NODELET_INFO_STREAM("Advertized on topic " << gray_topic);
    pub_depth = it_mynteye.advertiseCamera(depth_topic, 1);  // depth
    NODELET_INFO_STREAM("Advertized on topic " << depth_topic);
    pub_points = nh.advertise<sensor_msgs::PointCloud2>(points_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << points_topic);

    pub_imu = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << imu_topic);
    pub_temp = nh.advertise<mynteye_wrapper::Temp>(temp_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << temp_topic);

    double cx, cy, fx, fy;
    std::int32_t points_frequency;
    nh_ns.getParam("cx", cx);
    nh_ns.getParam("cy", cy);
    nh_ns.getParam("fx", fx);
    nh_ns.getParam("fy", fy);
    nh_ns.getParam("points_frequency", points_frequency);

    pointcloud_generator.reset(new PointCloudGenerator(
      {
        1000.0,  // factor, mm > m
        cx > 0 ? cx : 682.3,  // cx
        cy > 0 ? cy : 254.9,  // cy
        fx > 0 ? fx : 979.8,  // fx
        fy > 0 ? fy : 942.8,  // fy
      },
      [this](sensor_msgs::PointCloud2 msg) {
        // msg.header.seq = 0;
        // msg.header.stamp = ros::Time::now();
        msg.header.frame_id = points_frame_id;
        pub_points.publish(msg);
        // NODELET_INFO_STREAM("Publish points");
      }, points_frequency));

    device_poll_thread = boost::shared_ptr<boost::thread>(new boost::thread(
        boost::bind(&MYNTEYEWrapperNodelet::device_poll, this)));
  }
};

MYNTEYE_END_NAMESPACE

#include <pluginlib/class_list_macros.h> // NOLINT
PLUGINLIB_EXPORT_CLASS(MYNTEYE_NAMESPACE::MYNTEYEWrapperNodelet, nodelet::Nodelet);
