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

  /**
   * 鼠标事件：默认不选中区域，随鼠标移动而显示。单击后，则会选中区域来显示。你可以再单击已选中区域或双击未选中区域，取消选中。
   */
  void OnMouse(const int& event, const int& x, const int& y, const int& flags) {
    if (event != cv::EVENT_MOUSEMOVE && event != cv::EVENT_LBUTTONDOWN) {
      return;
    }
    show_ = true;

    if (event == cv::EVENT_MOUSEMOVE) {
      if (!selected_) {
        point_.x = x;
        point_.y = y;
      }
    } else if (event == cv::EVENT_LBUTTONDOWN) {
      if (selected_) {
        if (x >= static_cast<int>(point_.x - n_) &&
            x <= static_cast<int>(point_.x + n_) &&
            y >= static_cast<int>(point_.y - n_) &&
            y <= static_cast<int>(point_.y + n_)) {
          selected_ = false;
        }
      } else {
        selected_ = true;
      }
      point_.x = x;
      point_.y = y;
    }
  }

  template<typename T>
  void ShowElems(const cv::Mat& depth,
      std::function<std::string(const T& elem)> elem2string,
      int elem_space = 40,
      std::function<std::string(const cv::Mat& depth, const cv::Point& point,
          const std::uint32_t& n)> getinfo = nullptr) {
    if (!show_) return;

    int space = std::move(elem_space);
    int n = 2 * n_ + 1;
    cv::Mat im(space*n, space*n, CV_8UC3, cv::Scalar(255,255,255));

    int x, y;
    std::string str;
    int baseline = 0;
    for (int i = -n_; i <= n; ++i) {
      x = point_.x + i;
      if (x < 0 || x >= depth.cols) continue;
      for (int j = -n_; j <= n; ++j) {
        y = point_.y + j;
        if (y < 0 || y >= depth.rows) continue;

        str = elem2string(depth.at<T>(y, x));

        cv::Scalar color(0,0,0);
        if (i == 0 && j == 0) color = cv::Scalar(0,0,255);

        cv::Size sz = cv::getTextSize(str,
          cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

        cv::putText(im, str,
          cv::Point((i+n_)*space + (space-sz.width)/2,
            (j+n_)*space + (space+sz.height)/2),
          cv::FONT_HERSHEY_PLAIN, 1, color, 1);
      }
    }

    if (getinfo) {
      std::string info = getinfo(depth, point_, n_);
      if (!info.empty()) {
        cv::Size sz = cv::getTextSize(info,
          cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

        cv::putText(im, info,
          cv::Point(5, 5 + sz.height),
          cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,0,255), 1);
      }
    }

    cv::imshow("region", im);
  }

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

void OnDepthMouseCallback(int event, int x, int y, int flags, void* userdata) {
  DepthRegion* region = reinterpret_cast<DepthRegion*>(userdata);
  region->OnMouse(event, x, y, flags);
}

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

  cv::namedWindow("color");
  cv::namedWindow("depth");
  cv::namedWindow("region");

  DepthRegion depth_region(3);
  auto depth_info = [](
      const cv::Mat& depth, const cv::Point& point, const std::uint32_t& n) {
    /*
    int row_beg = point.y - n, row_end = point.y + n + 1;
    int col_beg = point.x - n, col_end = point.x + n + 1;
    if (row_beg < 0) row_beg = 0;
    if (row_end >= depth.rows) row_end = depth.rows;
    if (col_beg < 0) col_beg = 0;
    if (col_end >= depth.cols) col_end = depth.cols;
    cout << "[" << point.y << ", " << point.x << "]" << endl
      << depth.rowRange(row_beg, row_end).colRange(col_beg, col_end)
      << endl << endl;
    */
    std::ostringstream os;
    os << "depth pos: [" << point.y << ", " << point.x << "]"
      << "±" << n << ", unit: mm";
    return os.str();
  };

  CVPainter painter;
  util::Counter counter;
  for (;;) {
    cam.WaitForStream();
    counter.Update();

    auto image_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
    if (image_color.img) {
      cv::Mat color = image_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
      painter.DrawSize(color, CVPainter::TOP_LEFT);
      painter.DrawStreamData(color, image_color, CVPainter::TOP_RIGHT);
      painter.DrawInformation(color, util::to_string(counter.fps()),
          CVPainter::BOTTOM_RIGHT);

      cv::setMouseCallback("color", OnDepthMouseCallback, &depth_region);
      depth_region.DrawRect(color);
      cv::imshow("color", color);
    }

    auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
    if (image_depth.img) {
      cv::Mat depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();

      cv::setMouseCallback("depth", OnDepthMouseCallback, &depth_region);
      // Note: DrawRect will change some depth values to show the rect.
      depth_region.DrawRect(depth);
      cv::imshow("depth", depth);

      depth_region.ShowElems<ushort>(depth, [](const ushort& elem) {
        return std::to_string(elem);
      }, 80, depth_info);
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
