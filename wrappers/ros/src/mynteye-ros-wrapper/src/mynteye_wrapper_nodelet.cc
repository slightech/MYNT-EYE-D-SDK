#include <string>
#include <unistd.h>
#include <vector>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/camera.h"

namespace mynteye_wrapper {

namespace enc = sensor_msgs::image_encodings;

class MYNTEYEWrapperNodelet : public nodelet::Nodelet {
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns;
  boost::shared_ptr<boost::thread> device_poll_thread;

  image_transport::Publisher pub_color;
  image_transport::Publisher pub_depth;
  ros::Publisher pub_points;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

  // Launch params
  int dev_index;
  int framerate;
  int depth_mode;
  int stream_mode;
  int stream_format;
  bool state_ae;
  bool state_awb;
  int ir_intensity;

  std::string base_frame_id;
  std::string color_frame_id;
  std::string depth_frame_id;
  std::string points_frame_id;

  // MYNTEYE objects
  mynteye::InitParams params;
  std::unique_ptr<mynteye::Camera> mynteye;

  std::string dashes;

 public:
  MYNTEYEWrapperNodelet() : dashes(std::string(30, '-')) {
  }

  void publishColor(cv::Mat img, ros::Time stamp) {
    std_msgs::Header header;
    // header.seq = 0;
    header.stamp = stamp;
    header.frame_id = color_frame_id;
    pub_color.publish(cv_bridge::CvImage(header, enc::BGR8, img).toImageMsg());
    // NODELET_INFO_STREAM("Publish color");
  }

  void publishDepth(cv::Mat img, ros::Time stamp) {
    std_msgs::Header header;
    // header.seq = 0;
    header.stamp = stamp;
    header.frame_id = depth_frame_id;
    if (depth_mode == 0) {  // DEPTH_NON
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::BGR8, img).toImageMsg());
    } else if (depth_mode == 1) {  // DEPTH_GRAY
      cv::cvtColor(img, img, CV_BGR2GRAY);
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::MONO8, img).toImageMsg());
    } else if (depth_mode == 2) {  // DEPTH_COLORFUL
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::BGR8, img).toImageMsg());
    } else if (depth_mode == 3) {  // DEPTH_NON_16UC1
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::MONO16, img).toImageMsg());
    } else if (depth_mode == 4) {  // DEPTH_NON_8UC1
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::MONO8, img).toImageMsg());
    } else {
      NODELET_ERROR_STREAM("Depth mode unsupported");
      return;
    }
    // NODELET_INFO_STREAM("Publish depth");
  }

  const float camera_factor = 1000.0;

  const double camera_cx = 682.3;
  const double camera_cy = 254.9;
  const double camera_fx = 979.8;
  const double camera_fy = 942.8;

  void publishPoints(cv::Mat img, cv::Mat depth, ros::Time stamp) {
    if (pub_points.getNumSubscribers() == 0)
      return;

    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = points_frame_id;
    msg.width = img.cols;
    msg.height = img.rows;
    msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg);

    modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
        sensor_msgs::PointField::FLOAT32, "z", 1,
        sensor_msgs::PointField::FLOAT32, "rgb", 1,
        sensor_msgs::PointField::FLOAT32);

    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

    for (int m = 0; m < depth.rows; m++) {
      for (int n = 0; n < depth.cols; n++) {
        // get depth value at (m, n)
        unsigned short d = depth.ptr<unsigned short>(m)[n];
        // when d is equal 0 or 4096 means no depth
        if (d == 0 || d == 4096)
          continue;

        *iter_z = float(d) / camera_factor;
        *iter_x = (n - camera_cx) * *iter_z / camera_fx;
        *iter_y = (m - camera_cy) * *iter_z / camera_fy;

        *iter_r = img.ptr<uchar>(m)[n * 3 + 2];
        *iter_g = img.ptr<uchar>(m)[n * 3 + 1];
        *iter_b = img.ptr<uchar>(m)[n * 3];

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
      }
    }

    pub_points.publish(msg);
  }

  void device_poll() {
    using namespace mynteye;

    mynteye->Open(params);
    if (!mynteye->IsOpened()) {
      NODELET_ERROR_STREAM("Open camera failed");
      return;
    }
    NODELET_INFO_STREAM("Open camera success");

    ros::Rate loop_rate(params.framerate);

    cv::Mat color, depth;
    // Main loop
    while (nh_ns.ok()) {
      // Check for subscribers
      int color_SubNumber = pub_color.getNumSubscribers();
      int depth_SubNumber = pub_depth.getNumSubscribers();
      bool runLoop = (color_SubNumber + depth_SubNumber) > 0;

      if (runLoop) {
        ros::Time t = ros::Time::now();

        if (color_SubNumber > 0 || depth_SubNumber > 0) {
          if (mynteye->RetrieveImage(&color, &depth) == ErrorCode::SUCCESS) {
            if (color_SubNumber > 0) {
              publishColor(color, t);
            }
            if (depth_SubNumber > 0) {
              publishDepth(depth, t);
            }
            publishPoints(color, depth, t);
          }
        }
      }
      loop_rate.sleep();
    }

    mynteye.reset();
  }

  void onInit() {
    nh = getMTNodeHandle();
    nh_ns = getMTPrivateNodeHandle();

    // Launch params
    dev_index = 0;
    framerate = 30;
    depth_mode = 0;
    stream_mode = 0;
    stream_format = 0;
    state_ae = true;
    state_awb = true;
    ir_intensity = 0;

    nh_ns.getParam("dev_index", dev_index);
    nh_ns.getParam("framerate", framerate);
    nh_ns.getParam("depth_mode", depth_mode);
    nh_ns.getParam("stream_mode", stream_mode);
    nh_ns.getParam("stream_format", stream_format);
    nh_ns.getParam("state_ae", state_ae);
    nh_ns.getParam("state_awb", state_awb);
    nh_ns.getParam("ir_intensity", ir_intensity);

    base_frame_id = "mynteye_link";
    color_frame_id = "mynteye_color";
    depth_frame_id = "mynteye_depth";
    points_frame_id = "mynteye_points";
    nh_ns.getParam("base_frame_id", base_frame_id);
    nh_ns.getParam("color_frame", color_frame_id);
    nh_ns.getParam("depth_frame", depth_frame_id);
    nh_ns.getParam("points_frame", points_frame_id);
    NODELET_INFO_STREAM("base_frame: " << base_frame_id);
    NODELET_INFO_STREAM("color_frame: " << color_frame_id);
    NODELET_INFO_STREAM("depth_frame: " << depth_frame_id);
    NODELET_INFO_STREAM("points_frame: " << points_frame_id);

    std::string color_topic = "mynteye/color";
    std::string depth_topic = "mynteye/depth";
    std::string points_topic = "mynteye/points";
    nh_ns.getParam("color_topic", color_topic);
    nh_ns.getParam("depth_topic", depth_topic);
    nh_ns.getParam("points_topic", points_topic);

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
    params.stream_format = static_cast<mynteye::StreamFormat>(stream_format);
    params.state_ae = state_ae;
    params.state_awb = state_awb;
    params.ir_intensity = ir_intensity;

    // Image publishers
    image_transport::ImageTransport it_mynteye(nh);
    pub_color = it_mynteye.advertise(color_topic, 1);  // color
    NODELET_INFO_STREAM("Advertized on topic " << color_topic);
    pub_depth = it_mynteye.advertise(depth_topic, 1);  // depth
    NODELET_INFO_STREAM("Advertized on topic " << depth_topic);
    pub_points = nh.advertise<sensor_msgs::PointCloud2>(points_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << points_topic);

    device_poll_thread = boost::shared_ptr<boost::thread>(
        new boost::thread(
            boost::bind(&MYNTEYEWrapperNodelet::device_poll, this)));
  }

};

}  // namespace mynteye_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mynteye_wrapper::MYNTEYEWrapperNodelet, nodelet::Nodelet);
