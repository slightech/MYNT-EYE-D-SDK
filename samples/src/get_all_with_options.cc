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

#include "mynteye/camera.h"
#include "mynteye/utils.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"
#include "util/optparse.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char const* argv[]) {
  optparse::OptionParser parser = optparse::OptionParser()
      .usage("usage: %prog [options]"
      "\n  help: %prog -h"
      "\n  left+depth: %prog"
      "\n  imu only: %prog -r -m"
      "\n            -r for right, but default stream mode not support right"
      )
      .description("Open device with different options.");

  // OpenParams
  optparse::OptionGroup op_group = optparse::OptionGroup(
      "Open Params", "The open params");
  op_group.add_option("-i", "--index").dest("index")
      .type("int").metavar("INDEX").help("Device index");
  op_group.add_option("-f", "--rate").dest("framerate")
      .type("int").set_default(10)
      .metavar("RATE").help("Framerate, range [0,60], [0,30](STREAM_2560x720), "
          "\ndefault: %default");
  op_group.add_option("--cm").dest("color_mode")
      .type("int").set_default(0)
      .metavar("MODE").help("Color mode, default %default (COLOR_RAW)"
          "\n  0: COLOR_RAW, color raw"
          "\n  1: COLOR_RECTIFIED, color rectified");
  op_group.add_option("--dm").dest("depth_mode")
      .type("int").set_default(2)
      .metavar("MODE").help("Depth mode, default %default (DEPTH_COLORFUL)"
          "\n  0: DEPTH_RAW"
          "\n  1: DEPTH_GRAY"
          "\n  2: DEPTH_COLORFUL");
  op_group.add_option("--sm").dest("stream_mode")
      .type("int").set_default(2)
      .metavar("MODE").help("Stream mode of color & depth, "
          "\ndefault %default (STREAM_1280x720)"
          "\n  0: STREAM_640x480, 480p, vga, left"
          "\n  1: STREAM_1280x480, 480p, vga, left+right"
          "\n  2: STREAM_1280x720, 720p, hd, left"
          "\n  3: STREAM_2560x720, 720p, hd, left+right");
  op_group.add_option("--dev-mode").dest("device_mode")
      .type("int").set_default(2)
      .metavar("MODE").help("Device mode, default %default (ALL_DEVICE)"
          "\n  0: COLOR_DEVICE"
          "\n  1: DEPTH_DEVICE"
          "\n  2: ALL_DEVICE");
  op_group.add_option("--ae").dest("state_ae")
      .action("store_true").help("Enable auto-exposure");
  op_group.add_option("--awb").dest("state_awb")
      .action("store_true").help("Enable auto-white balance");
  op_group.add_option("--ir").dest("ir_intensity")
      .type("int").set_default(0)
      .metavar("VALUE").help("IR intensity, range [0,6], default %default");
  parser.add_option_group(op_group);

  // FeatureToggles
  optparse::OptionGroup ft_group = optparse::OptionGroup(
      "Feature Toggles", "The feature toggles");
  ft_group.add_option("--proc").dest("proc_mode")
      .type("int").set_default(0)
      .metavar("MODE").help("Enable process mode, e.g. imu assembly, temp_drift"
          "\n  0: PROC_NONE"
          "\n  1: PROC_IMU_ASSEMBLY"
          "\n  2: PROC_IMU_TEMP_DRIFT"
          "\n  3: PROC_IMU_ALL");
  ft_group.add_option("--img-info").dest("img_info")
      .action("store_true").help("Enable image info, and sync with image");
  parser.add_option_group(ft_group);

  // Streams & motions
  parser.add_option("-l", "--left").dest("left")
      .action("store_true").help("Enable left color stream");
  parser.add_option("-r", "--right").dest("right")
      .action("store_true").help("Enable right color stream");
  parser.add_option("-d", "--depth").dest("depth")
      .action("store_true").help("Enable depth stream");
  parser.add_option("-m", "--imu").dest("imu")
      .action("store_true").help("Enable imu datas");

  auto&& options = parser.parse_args(argc, argv);
  // auto&& args = parser.args();

  std::string dashes(80, '-');

  Camera cam;
  OpenParams params;

  DeviceInfo dev_info;
  {
    std::vector<DeviceInfo> dev_infos = cam.GetDeviceInfos();
    int n = dev_infos.size();
    if (n <= 0) {
      std::cerr << "Error: Device not found" << std::endl;
      return 1;
    }

    std::cout << dashes << std::endl;
    std::cout << "Index | Device Information" << std::endl;
    std::cout << dashes << std::endl;
    for (auto &&info : dev_infos) {
      std::cout << std::setw(5) << info.index << " | " << info << std::endl;
    }
    std::cout << dashes << std::endl;

    if (!options["index"].empty()) {
      int i = options.get("index");
      if (i < 0 || i >= n) {
        std::cerr << "Error: Device index (" << i << ") out of range"
            << std::endl;
        return 2;
      }
      dev_info = dev_infos[i];
    } else {  // auto select
      if (n == 1) {
        dev_info = dev_infos[0];
        std::cout << "Auto select a device to open, index: 0" << std::endl;
      } else {
        int i;
        while (true) {
          std::cout << "Please select a device to open, index: ";
          std::cin >> i;
          std::cout << std::endl;
          if (i >= 0 && i < n) {
            dev_info = dev_infos[i];
            break;
          }
          std::cerr << "Error: Device index (" << i << ") out of range"
              << std::endl;
        }
      }
    }
  }

  util::print_stream_infos(cam, dev_info.index);

  std::cout << "Open device: " << dev_info.index << ", "
      << dev_info.name << std::endl << std::endl;

  auto in_range = [&options](const std::string& name, int min, int max,
      int* val) {
    *val = static_cast<int>(options.get(name));
    if (*val < min || *val > max) {
      std::cerr << "Error: " << name << " out of range [" << min << "," << max
          << "]" << std::endl;
      return false;
    }
    return true;
  };
  {
    int val;
    params.dev_index = dev_info.index;

    if (!in_range("color_mode", 0, 1, &val)) return 2;
    params.color_mode = static_cast<ColorMode>(val);
    if (!in_range("depth_mode", 0, 2, &val)) return 2;
    params.depth_mode = static_cast<DepthMode>(val);
    if (!in_range("stream_mode", 0, 3, &val)) return 2;
    params.stream_mode = static_cast<StreamMode>(val);
    if (!in_range("device_mode", 0, 2, &val)) return 2;
    params.device_mode = static_cast<DeviceMode>(val);
    params.state_ae = options.get("state_ae");
    params.state_awb = options.get("state_awb");
    if (!in_range("ir_intensity", 0, 6, &val)) return 2;
    params.ir_intensity = val;

    if (params.stream_mode == StreamMode::STREAM_2560x720) {
      if (!in_range("framerate", 0, 30, &val)) return 2;
    } else {
      if (!in_range("framerate", 0, 60, &val)) return 2;
    }
    params.framerate = val;
  }
  bool is_right_ok = util::is_right_color_supported(params.stream_mode);
  bool left = options.get("left");
  bool right = options.get("right");
  bool depth = options.get("depth");
  {
    int val;

    if (!in_range("proc_mode", 0, 3, &val)) return 2;
    // Enable what process logics
    cam.EnableProcessMode(val);

    if (options.get("img_info")) {
      // Enable image infos
      cam.EnableImageInfo(true);
    }

    // Enable what stream datas: left_color, right_color, depth
    if (!left && !right && !depth) {
      if (is_right_ok) {
        left = right = depth = true;
        cam.EnableStreamData(ImageType::IMAGE_ALL);
      } else {
        left = depth = true;
        cam.EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
        cam.EnableStreamData(ImageType::IMAGE_DEPTH);
      }
    } else {
      if (left) cam.EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
      if (right && is_right_ok) {
        cam.EnableStreamData(ImageType::IMAGE_RIGHT_COLOR);
      } else {
        right = false;
        if (!left && !depth) {
          std::cout << "Warning: there is no stream datas wanted?" << std::endl;
        }
      }
      if (depth) cam.EnableStreamData(ImageType::IMAGE_DEPTH);
    }

    bool imu = options.get("imu");
    if (imu) {
      // Enable motion datas
      cam.EnableMotionDatas(0);

      // Set motion data callback
      cam.SetMotionCallback([](const MotionData& data) {
        if (data.imu->flag == MYNTEYE_IMU_ACCEL) {
          std::cout << "[accel] stamp: " << data.imu->timestamp
            << ", x: " << data.imu->accel[0]
            << ", y: " << data.imu->accel[1]
            << ", z: " << data.imu->accel[2]
            << ", temp: " << data.imu->temperature
            << std::endl;
        } else if (data.imu->flag == MYNTEYE_IMU_GYRO) {
          std::cout << "[gyro] stamp: " << data.imu->timestamp
            << ", x: " << data.imu->gyro[0]
            << ", y: " << data.imu->gyro[1]
            << ", z: " << data.imu->gyro[2]
            << ", temp: " << data.imu->temperature
            << std::endl;
        }
        std::cout << std::flush;
      });
    }
  }

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  if (left) cv::namedWindow("left color");
  if (right) cv::namedWindow("right color");
  if (depth) cv::namedWindow("depth");

  CVPainter painter;
  util::Counter counter;
  for (;;) {
    counter.Update();

    if (left) {
      auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
      if (left_color.img) {
        cv::Mat mat = left_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        painter.DrawSize(mat, CVPainter::TOP_LEFT);
        painter.DrawStreamData(mat, left_color, CVPainter::TOP_RIGHT);
        painter.DrawInformation(mat, util::to_string(counter.fps()),
            CVPainter::BOTTOM_RIGHT);
        cv::imshow("left color", mat);
      }
    }

    if (right) {
      auto right_color = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
      if (right_color.img) {
        cv::Mat mat = right_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        painter.DrawSize(mat, CVPainter::TOP_LEFT);
        painter.DrawStreamData(mat, right_color, CVPainter::TOP_RIGHT);
        cv::imshow("right color", mat);
      }
    }

    if (depth) {
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
