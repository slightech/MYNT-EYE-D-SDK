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
#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"

#define PRINT_COLORFUL

namespace {

void detect_imu_data_stamp(const MYNTEYE_NAMESPACE::ImuData& imu_data) {
  static std::uint32_t accel_stamp_last = imu_data.timestamp;
  static std::uint32_t gyro_stamp_last = imu_data.timestamp;

  std::uint32_t stamp = imu_data.timestamp;
  std::uint32_t stamp_diff;
  if (imu_data.flag == MYNTEYE_IMU_ACCEL) {
    stamp_diff = stamp - accel_stamp_last;
    accel_stamp_last = stamp;
  } else {
    stamp_diff = stamp - gyro_stamp_last;
    gyro_stamp_last = stamp;
  }

  if (stamp_diff <= 0) {
    // same or rollback
    if (stamp_diff == 0) {
      std::cout
#ifdef PRINT_COLORFUL
          << "\033[1;33m"
#endif
          << "[SAME";
    } else {
      std::cout
#ifdef PRINT_COLORFUL
          << "\033[1;31m"
#endif
          << "[ROLLBACK";
    }
    std::cout << "]["
        << (imu_data.flag == MYNTEYE_IMU_ACCEL ? "accel" : "gyro") << "]"
        << " stamp: " << stamp
        << ", diff: " << stamp_diff
#ifdef PRINT_COLORFUL
        << "\033[0m"
#endif
        << std::endl;
  }
}

void detect_img_info_stamp(const MYNTEYE_NAMESPACE::ImgInfo& img_info) {
  static std::uint32_t stamp_last = img_info.timestamp;
  auto stamp = img_info.timestamp;
  auto stamp_diff = stamp - stamp_last;
  stamp_last = stamp;

  if (stamp_diff <= 0) {
    // same or rollback
    if (stamp_diff == 0) {
      std::cout
#ifdef PRINT_COLORFUL
          << "\033[1;33m"
#endif
          << "[SAME";
    } else {
      std::cout
#ifdef PRINT_COLORFUL
          << "\033[1;31m"
#endif
          << "[ROLLBACK";
    }
    std::cout << "]"
        << "[img_info] fid: " << img_info.frame_id
        << ", stamp: " << stamp
        << ", diff: " << stamp_diff
#ifdef PRINT_COLORFUL
        << "\033[0m"
#endif
        << std::endl;
  }
}

}  // namespace

MYNTEYE_USE_NAMESPACE

int main(int argc, char const* argv[]) {
  Camera cam;
  DeviceInfo dev_info;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  util::print_stream_infos(cam, dev_info.index);

  std::cout << "Open device: " << dev_info.index << ", "
      << dev_info.name << std::endl << std::endl;

  OpenParams params(dev_info.index);
  {
    // Framerate: 10(default), [0,60], [0,30](STREAM_2560x720)
    params.framerate = 10;

    // Color mode: raw(default), rectified
    // params.color_mode = ColorMode::COLOR_RECTIFIED;

    // Depth mode: colorful(default), gray, raw
    // params.depth_mode = DepthMode::DEPTH_GRAY;

    // Stream mode: left color only
    // params.stream_mode = StreamMode::STREAM_640x480;  // vga
    // params.stream_mode = StreamMode::STREAM_1280x720;  // hd
    // Stream mode: left+right color
    // params.stream_mode = StreamMode::STREAM_1280x480;  // vga
    params.stream_mode = StreamMode::STREAM_2560x720;  // hd

    // Infrared intensity: 0(default), [0,10]
    // params.ir_intensity = 4;
  }

  // Enable image infos
  cam.EnableImageInfo(false);

  // Enable motion datas
  cam.EnableMotionDatas(0);

  // Callbacks
  {
    // Set image info callback
    cam.SetImgInfoCallback([](const std::shared_ptr<ImgInfo>& info) {
      detect_img_info_stamp(*info);
    });

    // Set motion data callback
    cam.SetMotionCallback([](const MotionData& data) {
      detect_imu_data_stamp(*(data.imu));
    });
  }

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  bool is_left_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR);
  bool is_right_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR);
  bool is_depth_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_DEPTH);

  if (is_left_ok) cv::namedWindow("left color");
  if (is_right_ok) cv::namedWindow("right color");
  if (is_depth_ok) cv::namedWindow("depth");

  for (;;) {
    if (is_left_ok) {
      auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
      if (left_color.img) {
        cv::Mat left = left_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        cv::imshow("left color", left);
      }
    }

    if (is_right_ok) {
      auto right_color = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
      if (right_color.img) {
        cv::Mat right = right_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        cv::imshow("right color", right);
      }
    }

    if (is_depth_ok) {
      auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
      if (image_depth.img) {
        cv::Mat depth = image_depth.img->To(ImageFormat::DEPTH_BGR)->ToMat();
        cv::imshow("depth", depth);
      }
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  cam.Close();
  cv::destroyAllWindows();
  return 0;
}
