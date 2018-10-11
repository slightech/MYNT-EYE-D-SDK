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
#include <iomanip>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include "camera.h"
#include "dataset.h"
#include "times.h"

using namespace std;
using namespace mynteye;

int main(int argc, char const *argv[]) {
	string dashes(80, '-');

	Camera cam;

	DeviceInfo dev_info;
	{
		vector<DeviceInfo> dev_infos = cam.GetDevices();
		size_t n = dev_infos.size();
		if (n <= 0) {
			cerr << "Error: Device not found" << endl;
			return 1;
		}

		cout << dashes << endl;
		cout << "Index | Device Information" << endl;
		cout << dashes << endl;
		for (auto &&info : dev_infos) {
			cout << setw(5) << info.index << " | " << info << endl;
		}
		cout << dashes << endl;

		if (n <= 2) {
			dev_info = dev_infos[0];
			cout << "Auto select a device to open, index: 0"<< endl;
		} else {
			size_t i;
			cout << "Please select a device to open, index: ";
			cin >> i;
			cout << endl;
			if (i >= n) {
				cerr << "Error: Index out of range" << endl;
				return 1;
			}
			dev_info = dev_infos[i];
		}
	}

	{
		vector<StreamInfo> color_infos;
		vector<StreamInfo> depth_infos;
		cam.GetResolutions(dev_info.index, color_infos, depth_infos);

		cout << dashes << endl;
		cout << "Index | Color Stream Information" << endl;
		cout << dashes << endl;
		for (auto &&info : color_infos) {
			cout << setw(5) << info.index << " | " << info << endl;
		}
		cout << dashes << endl << endl;

		cout << dashes << endl;
		cout << "Index | Depth Stream Information" << endl;
		cout << dashes << endl;
		for (auto &&info : depth_infos) {
			cout << setw(5) << info.index << " | " << info << endl;
		}
		cout << dashes << endl << endl;
	}

	cout << "Open device: " << dev_info.index << ", " << dev_info.name << endl << endl;

	// Warning: Color stream format MJPG doesn't work.
	InitParams params(dev_info.index);
	params.depth_mode = DepthMode::DEPTH_COLORFUL;
	// params.stream_mode = StreamMode::STREAM_1280x720;
	params.ir_intensity = 4;

  // output file path
  const char *outdir;
  if (argc >= 2) {
    outdir = argv[1];
  } else {
    outdir = "./dataset";
  }
  d1000_tools::Dataset dataset(outdir);

	cam.Open(params, Source::ALL);

	cout << endl;
	if (!cam.IsOpened()) {
		cerr << "Error: Open camera failed" << endl;
		return 1;
	}
	cout << "Open device success" << endl << endl;

	cout << "Press ESC/Q on Windows to terminate" << endl;

	cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);

	double t, fps = 0;
	cv::Mat color, depth;
  std::size_t imu_count = 0;
  auto &&time_beg = times::now();
	for (;;) {
		t = (double)cv::getTickCount();
    auto &&motion_data = cam.GetMotionData();
    imu_count += motion_data.size();
    for (auto &&motion : motion_data) {
      if (3 != motion.imu->flag) {
        dataset.SaveMotionData(motion);
      } else {
        dataset.SaveStreamData(motion);
      }
    }
    std::cout << "\rSaved " << ", " << imu_count << " imus" << std::flush;


		if (cam.RetrieveImage(color, depth) == ErrorCode::SUCCESS) {
			cv::imshow("color", color);
			cv::imshow("depth", depth);
		}

		char key = (char)cv::waitKey(10);
		if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
			break;
		}

		t = (double)cv::getTickCount() - t;
		fps = cv::getTickFrequency() / t;
	}
  std::cout << " to " << outdir << std::endl;
  auto &&time_end = times::now();
	(void)(fps);

	cam.Close();

  float elapsed_ms =
    times::count<times::microseconds>(time_end - time_beg) * 0.001f;
  cout << "Time beg: " << times::to_local_string(time_beg)
    << ", end: " << times::to_local_string(time_end)
    << ", cost: " << elapsed_ms << "ms" << endl;
//  LOG(INFO) << "Img count: " << img_count
 //   << ", fps: " << (1000.f * img_count / elapsed_ms);
  cout << "Imu count: " << imu_count
    << ", hz: " << (1000.f * imu_count / elapsed_ms) << endl;

	cv::destroyAllWindows();
	return 0;
}
