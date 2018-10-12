#include <string>
#include <unistd.h>
#include <vector>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/camera.h"

#include "pointcloud_generator.h"

namespace mynteye_wrapper {

namespace enc = sensor_msgs::image_encodings;

class MYNTEYEWrapperNodelet : public nodelet::Nodelet {
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns;
  boost::shared_ptr<boost::thread> device_poll_thread;

  image_transport::Publisher pub_color;
  image_transport::Publisher pub_depth;
  ros::Publisher pub_points;
  // tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

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

  std::string base_frame_id;
  std::string color_frame_id;
  std::string depth_frame_id;
  std::string points_frame_id;

  // MYNTEYE objects
  mynteye::InitParams params;
  std::unique_ptr<mynteye::Camera> mynteye;

  std::unique_ptr<PointCloudGenerator> pointcloud_generator;

  std::string dashes;

 public:
  MYNTEYEWrapperNodelet() : dashes(std::string(30, '-')) {
  }

  void publishColor(mynteye::Image::pointer img, ros::Time stamp,
      cv::Mat* mat) {
    std_msgs::Header header;
    // header.seq = 0;
    header.stamp = stamp;
    header.frame_id = color_frame_id;
    *mat = img->To(mynteye::ImageFormat::COLOR_RGB)->ToMat();
    pub_color.publish(cv_bridge::CvImage(header, enc::RGB8, *mat).toImageMsg());
    // NODELET_INFO_STREAM("Publish color");
  }

  void publishDepth(mynteye::Image::pointer img, ros::Time stamp,
      cv::Mat* mat) {
    std_msgs::Header header;
    // header.seq = 0;
    header.stamp = stamp;
    header.frame_id = depth_frame_id;
    if (depth_mode == 0) {  // DEPTH_RAW
      *mat = img->To(mynteye::ImageFormat::DEPTH_RAW)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::MONO16, *mat).toImageMsg());
    } else if (depth_mode == 1) {  // DEPTH_GRAY
      *mat = img->To(mynteye::ImageFormat::DEPTH_GRAY_24)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::RGB8, *mat).toImageMsg());
    } else if (depth_mode == 2) {  // DEPTH_COLORFUL
      *mat = img->To(mynteye::ImageFormat::DEPTH_RGB)->ToMat();
      pub_depth.publish(
          cv_bridge::CvImage(header, enc::RGB8, *mat).toImageMsg());
    } else {
      NODELET_ERROR_STREAM("Depth mode unsupported");
      return;
    }
    // NODELET_INFO_STREAM("Publish depth");
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
    cv::Mat color, depth;
    while (nh_ns.ok()) {
      // Check for subscribers
      int color_SubNumber = pub_color.getNumSubscribers();
      int depth_SubNumber = pub_depth.getNumSubscribers();
      int points_SubNumber = pub_points.getNumSubscribers();

      bool runLoop = (color_SubNumber + depth_SubNumber + points_SubNumber) > 0;
      if (runLoop) {
        // publish points, the depth mode must be DEPTH_RAW.
        bool points_subscribed = (points_SubNumber > 0) && (depth_mode == 0);

        ros::Time t = ros::Time::now();

        bool color_ok = false;
        if (color_SubNumber > 0 || points_subscribed) {
          auto image_color = mynteye->RetrieveImage(
              mynteye::ImageType::IMAGE_COLOR);
          if (image_color.img) {
            color_ok = true;
            publishColor(image_color.img, t, &color);
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
    framerate = 10;
    depth_mode = 0;
    stream_mode = 0;
    color_stream_format = 0;
    depth_stream_format = 0;
    state_ae = true;
    state_awb = true;
    ir_intensity = 0;

    nh_ns.getParam("dev_index", dev_index);
    nh_ns.getParam("framerate", framerate);
    nh_ns.getParam("depth_mode", depth_mode);
    nh_ns.getParam("stream_mode", stream_mode);
    nh_ns.getParam("color_stream_format", color_stream_format);
    nh_ns.getParam("depth_stream_format", depth_stream_format);
    nh_ns.getParam("state_ae", state_ae);
    nh_ns.getParam("state_awb", state_awb);
    nh_ns.getParam("ir_intensity", ir_intensity);

    base_frame_id = "mynteye_link";
    color_frame_id = "mynteye_color_frame";
    depth_frame_id = "mynteye_depth_frame";
    points_frame_id = "mynteye_points_frame";
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
    params.color_stream_format =
        static_cast<mynteye::StreamFormat>(color_stream_format);
    params.depth_stream_format =
        static_cast<mynteye::StreamFormat>(depth_stream_format);
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

}  // namespace mynteye_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mynteye_wrapper::MYNTEYEWrapperNodelet, nodelet::Nodelet);
