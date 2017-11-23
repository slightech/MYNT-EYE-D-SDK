#include <string>
#include <vector>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camera.h"

namespace mynteye_wrapper {

namespace enc = sensor_msgs::image_encodings;

class MYNTEYEWrapperNodelet : public nodelet::Nodelet {

    ros::NodeHandle nh;
    ros::NodeHandle nh_ns;
    boost::shared_ptr<boost::thread> device_poll_thread;

    image_transport::Publisher pub_color;
    image_transport::Publisher pub_depth;

    // Launch params
    int dev_index;
    int framerate;
    int depth_mode;
    int color_index;
    int depth_index;
    bool state_ae;
    bool state_awb;
    int ir_intensity;

    std::string color_frame_id;
    std::string depth_frame_id;

    // MYNTEYE objects
    mynteye::InitParams params;
    std::unique_ptr<mynteye::Camera> mynteye;

    std::string dashes;

public:
    MYNTEYEWrapperNodelet() : dashes(std::string(30, '-')) {
    }

    void publishColor(cv::Mat img, ros::Time stamp) {
        std_msgs::Header header;
        //header.seq = 0;
        header.stamp = stamp;
        header.frame_id = color_frame_id;
        pub_color.publish(cv_bridge::CvImage(header, enc::BGR8, img).toImageMsg());
        //NODELET_INFO_STREAM("Publish color");
    }

    void publishDepth(cv::Mat img, ros::Time stamp) {
        std_msgs::Header header;
        //header.seq = 0;
        header.stamp = stamp;
        header.frame_id = depth_frame_id;
        if (depth_mode == 0) {  // DEPTH_NON
            pub_depth.publish(cv_bridge::CvImage(header, enc::MONO8, img).toImageMsg());
        } else if (depth_mode == 1) {  // DEPTH_GRAY
            cv::cvtColor(img, img, CV_BGR2GRAY);
            pub_depth.publish(cv_bridge::CvImage(header, enc::MONO8, img).toImageMsg());
        } else if (depth_mode == 2) {  // DEPTH_COLORFUL
            pub_depth.publish(cv_bridge::CvImage(header, enc::BGR8, img).toImageMsg());
        } else {
            NODELET_ERROR_STREAM("Depth mode unsupported");
            return;
        }
        //NODELET_INFO_STREAM("Publish depth");
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
                    if (mynteye->RetrieveImage(color, depth) == ErrorCode::SUCCESS) {
                        if (color_SubNumber > 0) {
                            publishColor(color, t);
                        }
                        if (depth_SubNumber > 0) {
                            publishDepth(depth, t);
                        }
                    }
                }

                loop_rate.sleep();
            }
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
        color_index = 0;
        depth_index = 0;
        state_ae = true;
        state_awb = true;
        ir_intensity = 0;

        nh_ns.getParam("dev_index", dev_index);
        nh_ns.getParam("framerate", framerate);
        nh_ns.getParam("depth_mode", depth_mode);
        nh_ns.getParam("color_index", color_index);
        nh_ns.getParam("depth_index", depth_index);
        nh_ns.getParam("state_ae", state_ae);
        nh_ns.getParam("state_awb", state_awb);
        nh_ns.getParam("ir_intensity", ir_intensity);

        color_frame_id = "mynteye_color";
        depth_frame_id = "mynteye_depth";
        nh_ns.getParam("color_frame", color_frame_id);
        nh_ns.getParam("depth_frame", depth_frame_id);
        NODELET_INFO_STREAM("color_frame: " << color_frame_id);
        NODELET_INFO_STREAM("depth_frame: " << depth_frame_id);

        std::string color_topic = "mynteye/color";
        std::string depth_topic = "mynteye/depth";
        nh_ns.getParam("color_topic", color_topic);
        nh_ns.getParam("depth_topic", depth_topic);

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
            mynteye->GetResolutions(dev_index, color_infos, depth_infos);

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
        params.color_info_index = color_index;
        params.depth_info_index = depth_index;
        params.state_ae = state_ae;
        params.state_awb = state_awb;
        params.ir_intensity = ir_intensity;

        // Image publishers
        image_transport::ImageTransport it_mynteye(nh);
        pub_color = it_mynteye.advertise(color_topic, 1);  // color
        NODELET_INFO_STREAM("Advertized on topic " << color_topic);
        pub_depth = it_mynteye.advertise(depth_topic, 1);  // depth
        NODELET_INFO_STREAM("Advertized on topic " << depth_topic);

        device_poll_thread = boost::shared_ptr<boost::thread>
            (new boost::thread(boost::bind(&MYNTEYEWrapperNodelet::device_poll, this)));
    }

};

}  // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mynteye_wrapper::MYNTEYEWrapperNodelet, nodelet::Nodelet);
