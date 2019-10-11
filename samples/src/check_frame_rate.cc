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
#include <fstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"

MYNTEYE_USE_NAMESPACE

template<typename info>
class Record {
 public:
  explicit Record(const std::string &filename, bool log): log_to_console(log) {
    file.open(filename);
    if (!file.is_open()) {
      std::cout << "file open failed:" << "./_output/"+filename << std::endl;
    }
    record_nums = 0;
  }

  virtual ~Record() {
    file << std::endl;
    for (int i = 0; i < 100; i++) {
      file << "_";
    }
    file << std::endl << "record_nums:" << record_nums << std::endl;
    file << std::endl << "error_nums:" << errors() << std::endl;
    for (auto err : err_infos) {
      file << err << std::endl;
    }
    file.close();
    // std::cout << "~Record()" << std::endl;
  }
  void record_fps(int fps) {
    file << "---------- fps:" << fps << " ----------" << std::endl;
     if (log_to_console)
        std::cout << "---------- fps:" << fps << " ----------" << std::endl;
  }
  void push_back(std::string pre, info new_info) {
    file << pre;
    if (log_to_console)
      std::cout << pre;
    push_back(new_info);
  }
  void push_back(info new_info) {
    if (record_nums) {
      if (last_info > new_info) {
        err_infos.push_back(new_info);
        if (log_to_console)
          std::cout << "error------------";
        file << "error";
        for (int i = 0; i < 50; i++) {
          file << "-";
        }
      }
    }
    file << new_info;
     if (log_to_console)
        std::cout << new_info;
    last_info = new_info;
    record_nums++;
  }
  int errors() {
    return err_infos.size();
  }

 private:
  std::ofstream file;
  bool log_to_console;
  info last_info;
  std::vector<info> err_infos;
  int record_nums;
};

int main(int argc, char const* argv[]) {
  std::string fn = "_";
  if (argc != 2) {
      std::cout << "Run the program with record filename param,"
                << "eg: ./check_frame_rate 190830" << std::endl
                << "use default name:out"
                << std::endl;
      fn = "out" + fn;
  } else {
      fn = argv[1] + fn;
  }
  Record<std::uint32_t> imu_record(fn + "imu_record.txt", false);

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
    // Framerate: 10(default usb3.0) 5(default usb2.0),
    // [0,60], [30](STREAM_2560x720)

    params.framerate = 10;
    std::cout << "Set rate to:" << params.framerate << std::endl;
    // Device mode, default DEVICE_ALL
    //   DEVICE_COLOR: IMAGE_LEFT_COLOR ✓ IMAGE_RIGHT_COLOR ? IMAGE_DEPTH x
    //   DEVICE_DEPTH: IMAGE_LEFT_COLOR x IMAGE_RIGHT_COLOR x IMAGE_DEPTH ✓
    //   DEVICE_ALL:   IMAGE_LEFT_COLOR ✓ IMAGE_RIGHT_COLOR ? IMAGE_DEPTH ✓
    // Note: ✓: available, x: unavailable, ?: depends on #stream_mode
    // params.dev_mode = DeviceMode::DEVICE_ALL;

    // Color mode: raw(default), rectified
    // params.color_mode = ColorMode::COLOR_RECTIFIED;

    // Depth mode: colorful(default), gray, raw
    // params.depth_mode = DepthMode::DEPTH_GRAY;

    // Stream mode: left color only
    // params.stream_mode = StreamMode::STREAM_640x480;  // vga
    params.stream_mode = StreamMode::STREAM_1280x720;  // hd
    // Stream mode: left+right color
    // params.stream_mode = StreamMode::STREAM_1280x480;  // vga
    // params.stream_mode = StreamMode::STREAM_2560x720;  // hd

    // Auto-exposure: true(default), false
    // params.state_ae = false;

    // Auto-white balance: true(default), false
    // params.state_awb = false;

    // IR Depth Only: true, false(default)
    // Note: IR Depth Only mode support frame rate between 15fps and 30fps.
    //     When dev_mode != DeviceMode::DEVICE_ALL,
    //       IR Depth Only mode not be supported.
    //     When stream_mode == StreamMode::STREAM_2560x720,
    //       frame rate only be 15fps in this mode.
    //     When frame rate less than 15fps or greater than 30fps,
    //       IR Depth Only mode will be not available.
    // params.ir_depth_only = true;

    // Infrared intensity: 0(default), [0,10]
    params.ir_intensity = 0;

    // Colour depth image, default 5000. [0, 16384]
    params.colour_depth_value = 0;
  }

  // Enable what process logics
  // cam.EnableProcessMode(ProcessMode::PROC_IMU_ALL);

  // Enable image infos
  cam.EnableImageInfo(true);

  // Enable motion datas
  cam.EnableMotionDatas();

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  bool is_left_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR);
  bool is_right_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR);
  bool is_depth_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_DEPTH);

  if (!cam.IsMotionDatasSupported()) {
    std::cerr << "Error: IMU is not supported on your device." << std::endl;
    return -1;
  }

  // std::string left_re = "left_record.txt";
  Record<ImgInfo> left_record(fn + "left_record.txt", true);
  Record<ImgInfo> right_record(fn + "right_record.txt", false);
  Record<ImgInfo> depth_record(fn + "depth_record.txt", false);
  if (is_left_ok) {
    cv::namedWindow("left color");
  }
  if (is_right_ok) {
    cv::namedWindow("right color");
  }
  if (is_left_ok) {
    cv::namedWindow("depth");
  }


  CVPainter painter;
  util::Counter counter;
  for (;;) {
    cam.WaitForStream();
    counter.Update();

    if (is_left_ok) {
      if (counter.count() % 30 == 0) {
        left_record.record_fps(counter.fps());
      }
      auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
      if (left_color.img) {
        cv::Mat left = left_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        painter.DrawSize(left, CVPainter::TOP_LEFT);
        painter.DrawStreamData(left, left_color, CVPainter::TOP_RIGHT);
        painter.DrawInformation(left, util::to_string(counter.fps()),
            CVPainter::BOTTOM_RIGHT);
        left_record.push_back(*(left_color.img_info));
        cv::imshow("left color", left);
      }
    }

    if (is_right_ok) {
      if (counter.count() % 30 == 0) {
        right_record.record_fps(counter.fps());
      }
      auto right_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
      if (right_color.img) {
        cv::Mat right = right_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        painter.DrawSize(right, CVPainter::TOP_LEFT);
        painter.DrawStreamData(right, right_color, CVPainter::TOP_RIGHT);
        painter.DrawInformation(right, util::to_string(counter.fps()),
            CVPainter::BOTTOM_RIGHT);
        right_record.push_back(*(right_color.img_info));
        cv::imshow("right color", right);
      }
    }

    if (is_depth_ok) {
      if (counter.count() % 30 == 0) {
        depth_record.record_fps(counter.fps());
      }
      auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
      if (image_depth.img) {
        cv::Mat depth;
        if (params.depth_mode == DepthMode::DEPTH_COLORFUL) {
          depth = image_depth.img->To(ImageFormat::DEPTH_BGR)->ToMat();
        } else {
          depth = image_depth.img->ToMat();
        }
        painter.DrawSize(depth, CVPainter::TOP_LEFT);
        painter.DrawStreamData(depth, image_depth, CVPainter::TOP_RIGHT);
        depth_record.push_back(*(image_depth.img_info));
        cv::imshow("depth", depth);
      }
    }

    auto motion_datas = cam.GetMotionDatas();
    if (motion_datas.size() > 0) {
      std::cout << "Imu count: " << motion_datas.size() << std::endl;
      for (auto data : motion_datas) {
        if (data.imu) {
          if (data.imu->flag == MYNTEYE_IMU_ACCEL) {
            imu_record.push_back("\n[accel] stamp: ", data.imu->timestamp);
            counter.IncrAccelCount();
            // std::cout << "[accel] stamp: " << data.imu->timestamp
            //   << ", x: " << data.imu->accel[0]
            //   << ", y: " << data.imu->accel[1]
            //   << ", z: " << data.imu->accel[2]
            //   << ", temp: " << data.imu->temperature
            //   << std::endl;
          } else if (data.imu->flag == MYNTEYE_IMU_GYRO) {
            imu_record.push_back("\n[gyro] stamp: ", data.imu->timestamp);
            counter.IncrGyroCount();
            // std::cout << "[gyro ] stamp: " << data.imu->timestamp
            //   << ", x: " << data.imu->gyro[0]
            //   << ", y: " << data.imu->gyro[1]
            //   << ", z: " << data.imu->gyro[2]
            //   << ", temp: " << data.imu->temperature
            //   << std::endl;
          } else {
            std::cerr << "Imu type is unknown" << std::endl;
          }
        } else {
          std::cerr << "Motion data is empty" << std::endl;
        }
      }
      std::cout << std::endl;
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }
  cam.Close();
  counter.PrintCountInfo();
  cv::destroyAllWindows();
  return 0;
}
