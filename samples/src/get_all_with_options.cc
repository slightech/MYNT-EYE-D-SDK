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
      "\n  left+right+depth: %prog --sm=3"
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
  op_group.add_option("--dev-mode").dest("dev_mode")
      .type("int").set_default(2)
      .metavar("MODE").help("Device mode, default %default (DEVICE_ALL)"
          "\n  0: DEVICE_COLOR, left y right - depth n"
          "\n  1: DEVICE_DEPTH, left n right n depth y"
          "\n  2: DEVICE_ALL,   left y right - depth y"
          "\n  Note: y: available, n: unavailable, -: depends on stream mode");
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
  op_group.add_option("--csf").dest("color_stream_format")
      .type("int").set_default(1)
      .metavar("MODE").help("Stream format of color, "
          "\ndefault %default (STREAM_YUYV)"
          "\n  0: STREAM_MJPG"
          "\n  1: STREAM_YUYV");
  op_group.add_option("--dsf").dest("depth_stream_format")
      .type("int").set_default(1)
      .metavar("MODE").help("Stream format of depth, "
          "\ndefault %default (STREAM_YUYV)"
          "\n  1: STREAM_YUYV");
  op_group.add_option("--ae").dest("state_ae")
      .action("store_true").help("Enable auto-exposure");
  op_group.add_option("--awb").dest("state_awb")
      .action("store_true").help("Enable auto-white balance");
  op_group.add_option("--ir").dest("ir_intensity")
      .type("int").set_default(0)
      .metavar("VALUE").help("IR intensity, range [0,10], default %default");
  op_group.add_option("--ir-depth").dest("ir_depth_only")
      .action("store_false").help("Enable ir-depth-only");
  op_group.add_option("--cdv").dest("colour_depth_value")
      .type("float").set_default(1000)
      .metavar("VALUE").help("Colour depth value, "
          "range [0, 16384], default %default");
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
      .action("store_false").help("Enable image info, and sync with image");
  parser.add_option_group(ft_group);

  // Streams & motions
  parser.add_option("-m", "--imu").dest("imu")
      .action("store_true").help("Enable imu datas");

  // Others
  parser.add_option("--show-secs").dest("show_secs")
      .type("int").set_default(0)
      .metavar("SECONDS").help("The show seconds, default: %default");

  auto&& options = parser.parse_args(argc, argv);
  // auto&& args = parser.args();

  std::string dashes(80, '-');

  Camera cam;
  OpenParams params;
  util::Counter counter;

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

  int show_ms = 0;

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

    if (!in_range("dev_mode", 0, 2, &val)) return 2;
    params.dev_mode = static_cast<DeviceMode>(val);
    if (!in_range("color_mode", 0, 1, &val)) return 2;
    params.color_mode = static_cast<ColorMode>(val);
    if (!in_range("depth_mode", 0, 2, &val)) return 2;
    params.depth_mode = static_cast<DepthMode>(val);
    if (!in_range("stream_mode", 0, 3, &val)) return 2;
    params.stream_mode = static_cast<StreamMode>(val);
    if (!in_range("color_stream_format", 0, 1, &val)) return 2;
    params.color_stream_format = static_cast<StreamFormat>(val);
    if (!in_range("depth_stream_format", 1, 1, &val)) return 2;
    params.depth_stream_format = static_cast<StreamFormat>(val);
    params.state_ae = options.get("state_ae");
    params.state_awb = options.get("state_awb");
    if (!in_range("ir_intensity", 0, 10, &val)) return 2;
    params.ir_intensity = val;
    params.ir_depth_only = options.get("ir_depth_only");
    if (!in_range("colour_depth_value", 0, 16384, &val)) return 2;
    params.colour_depth_value = val;

    if (params.stream_mode == StreamMode::STREAM_2560x720 &&
        params.dev_mode == DeviceMode::DEVICE_ALL) {
      if (!in_range("framerate", 0, 30, &val)) return 2;
    } else {
      if (!in_range("framerate", 0, 60, &val)) return 2;
    }
    params.framerate = val;
  }
  {
    int val;

    if (!in_range("proc_mode", 0, 3, &val)) return 2;
    // Enable what process logics
    cam.EnableProcessMode(val);

    if (options.get("img_info")) {
      // Enable image infos
      cam.EnableImageInfo(true);
    }

    bool imu = options.get("imu");
    if (imu) {
      // Enable motion datas
      cam.EnableMotionDatas(0);

      // Set motion data callback
      cam.SetMotionCallback([&counter](const MotionData& data) {
        if (data.imu->flag == MYNTEYE_IMU_ACCEL) {
          counter.IncrAccelCount();
          std::cout << "[accel] stamp: " << data.imu->timestamp
            << ", x: " << data.imu->accel[0]
            << ", y: " << data.imu->accel[1]
            << ", z: " << data.imu->accel[2]
            << ", temp: " << data.imu->temperature
            << std::endl;
        } else if (data.imu->flag == MYNTEYE_IMU_GYRO) {
          counter.IncrGyroCount();
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

    show_ms = static_cast<int>(options.get("show_secs")) * 1000;
  }
  {
    cam.SetStreamCallback(ImageType::IMAGE_LEFT_COLOR, [&counter](
        const StreamData &data) {
      if (data.img) counter.IncrColorCount();
    });
    cam.SetStreamCallback(ImageType::IMAGE_DEPTH, [&counter](
        const StreamData &data) {
      if (data.img) counter.IncrDepthCount();
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

  CVPainter painter;
  for (;;) {
    cam.WaitForStream();
    counter.Update();

    if (is_left_ok) {
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

    if (is_right_ok) {
      auto right_color = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
      if (right_color.img) {
        cv::Mat mat = right_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        painter.DrawSize(mat, CVPainter::TOP_LEFT);
        painter.DrawStreamData(mat, right_color, CVPainter::TOP_RIGHT);
        cv::imshow("right color", mat);
      }
    }

    if (is_depth_ok) {
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

    if (show_ms > 0 && counter.ElapsedMillis() > show_ms) {
      break;  // timeout
    }
  }

  cam.Close();
  cv::destroyAllWindows();

  counter.PrintCountInfo();
  return 0;
}
