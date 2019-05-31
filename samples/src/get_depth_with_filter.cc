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
#include <functional>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"
#include "mynteyed/filter/spatial_filter.h"
#include "mynteyed/filter/temporal_filter.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"

namespace {

class DepthRegion {
 public:
  explicit DepthRegion(std::uint32_t n)
    : n_(std::move(n)),
      show_(false),
      selected_(false),
      point_(0, 0) {
  }

  ~DepthRegion() = default;

  void DrawRect(const cv::Mat &im) {
    if (!show_) return;
    std::uint32_t n = (n_ > 1) ? n_ : 1;
    n += 1;  // outside the region
#ifdef WITH_OPENCV2
    cv::rectangle(const_cast<cv::Mat&>(im),
#else
    cv::rectangle(im,
#endif
      cv::Point(point_.x-n, point_.y-n),
      cv::Point(point_.x+n, point_.y+n),
      selected_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 1);
  }

 private:
  std::uint32_t n_;
  bool show_;
  bool selected_;
  cv::Point point_;
};

}  // namespace

using namespace std;

MYNTEYE_USE_NAMESPACE

int main(int argc, char const* argv[]) {
  Camera cam;
  DeviceInfo dev_info;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  util::print_stream_infos(cam, dev_info.index);

  SpatialFilter spat_filter;
  TemporalFilter temp_filter;

  cout << "Open device: " << dev_info.index << ", "
      << dev_info.name << endl << endl;

  OpenParams params(dev_info.index);
  {
    // Framerate: 10(default), [0,60], [0,30](STREAM_2560x720)
    params.framerate = 10;

    // Color mode: raw(default), rectified
    // params.color_mode = ColorMode::COLOR_RECTIFIED;

    // Depth mode: colorful(default), gray, raw
    // Note: must set DEPTH_RAW to get raw depth values
    params.depth_mode = DepthMode::DEPTH_RAW;

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

    // Infrared intensity: 0(default), [0,10]
    params.ir_intensity = 4;
  }

  cam.Open(params);

  cout << endl;
  if (!cam.IsOpened()) {
    cerr << "Error: Open camera failed" << endl;
    return 1;
  }
  cout << "Open device success" << endl << endl;

  cout << "Press ESC/Q on Windows to terminate" << endl;

  DepthRegion depth_region(3);

  CVPainter painter;
  util::Counter counter;
  for (;;) {
    cam.WaitForStream();
    counter.Update();

    auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
    if (image_depth.img) {
      cv::Mat depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();
      // Note: DrawRect will change some depth values to show the rect.
      depth_region.DrawRect(depth);
      cv::Mat res_org;
      cv::normalize(depth, res_org, 0, 255, cv::NORM_MINMAX, CV_8UC1);
#ifdef WITH_OPENCV3
      // ColormapTypes
      //   http://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
      cv::applyColorMap(res_org, res_org, cv::COLORMAP_JET);
#endif
      cv::imshow("depth_before_filter", res_org);
      spat_filter.ProcessFrame(image_depth.img, image_depth.img);
      temp_filter.ProcessFrame(image_depth.img, image_depth.img);
      cv::Mat res;
      cv::normalize(depth, res, 0, 255, cv::NORM_MINMAX, CV_8UC1);
#ifdef WITH_OPENCV3
      // ColormapTypes
      //   http://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
      cv::applyColorMap(res, res, cv::COLORMAP_JET);
#endif
      cv::imshow("depth_after_filter", res);
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
