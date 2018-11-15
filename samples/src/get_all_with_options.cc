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
#include <limits.h>
#include <stdlib.h>
#include <iostream>
#include <functional>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/camera.h"
#include "mynteye/utils.h"
#include "mynteye/types.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"

using namespace std;
using namespace mynteye;

typedef struct prams {
  int default_val;
  string description;
  int min;
  int max;
  vector<string> options_description;
}PRAM;

PRAM config[] = {
  {0,   "device index",       0,  INT_MAX,
    {
      "0 device0",
      "1 device1",
      "..."
    }
  },
  {10,  "framerate",          1,  60,
    {
      "StreamMode STREAM_2560x720: <30",
      "StreamMode STREAM_640x480/STREAM_1280x480/STREAM_1280x720 <60"
    }
  },
  {0,   "color mode",         0,  static_cast<int>(ColorMode::COLOR_MODE_LAST),
    {
      "0 COLOR_RAW (color raw)",
      "1 COLOR_RECTIFIED (color rectified)"
    }
  },
  {2,   "depth mode",         0,  static_cast<int>(DepthMode::DEPTH_MODE_LAST),
    {
      "0 DEPTH_RAW (ImageFormat::DEPTH_RAW)",
      "1 DEPTH_GRAY (ImageFormat::DEPTH_GRAY_24)",
      "2 DEPTH_COLORFUL (ImageFormat::DEPTH_RGB)"
    }
  },
  {2,   "stream mode",        0,  static_cast<int>(StreamMode::STREAM_MODE_LAST), // NOLINT
    {
      "0 STREAM_640x480 (480p, vga, left+right)",
      "1 STREAM_1280x480 (480p, vga, left)",
      "2 STREAM_1280x720 (720p, hd, left)",
      "3 STREAM_2560x720 (720p, hd, left+right)"
    }
  },
  {1,   "color stream format", 0,  static_cast<int>(StreamFormat::STREAM_FORMAT_LAST),  // NOLINT
    {
      "0 STREAM_MJPG",
      "1 STREAM_YUYV"
    }
  },
  {1,   "depth stream format", 0,  static_cast<int>(StreamFormat::STREAM_FORMAT_LAST),  // NOLINT
    {
      "0 STREAM_MJPG",
      "1 STREAM_YUYV"
    }
  },
  {1,   "Auto-exposure",       0,  1,
    {
      "0 off",
      "1 on"
    }
  },
  {1,   "Auto-white balance",  0,  1,
    {
      "0 off",
      "1 on"
    }
  }
};

int main(int argc, char const* argv[]) {
  string dashes(std::string(30, '-'));
  mynteye::Camera cam;
  mynteye::OpenParams params;
  for (unsigned int i=0; i < sizeof(config)/sizeof(PRAM); i++) {
    cout << dashes << endl;
    bool use_default_tag = false;
    cout << config[i].description << endl;
    for (unsigned int j = 0; j < config[i].options_description.size(); j++) {
      cout << config[i].options_description[j] << endl;
    }
    if ((unsigned int)argc-1 <= i) {
      use_default_tag = true;
      cout << "use default value:"<< config[i].default_val << endl;
    }

    if (use_default_tag == false) {
      if (atoi(argv[i+1]) < config[i].min || atoi(argv[i+1]) > config[i].max) {
        use_default_tag = true;
        cout << "out of range and use default value:"
                << config[i].default_val << endl;
      }
    }

    switch (i) {
      case 0:
      {
        int dev_index = config[i].default_val;
        if (use_default_tag) {
          params = OpenParams(dev_index);
        } else {
          dev_index = atoi(argv[i+1]);
          params = OpenParams(dev_index);
        }
        {
          vector<mynteye::DeviceInfo> dev_infos = cam.GetDeviceInfos();
          size_t n = dev_infos.size();
          if (n <= 0 || dev_index < 0 || (unsigned int)dev_index >= n) {
            cout << "Device not found, index: " << dev_index << endl;
            return 1;
          }
          cout << "Device Information" << endl;
          int i = 0;
          for (auto &&info : dev_infos) {
            if (dev_index == i) {
              cout << "*";
            } else {
              cout << " ";
            }
            cout << info.index << " | " << info << endl;
            i++;
          }
        }
        break;
      }
      case 1:
      {
        if (use_default_tag) {
          params.framerate = config[i].default_val;
        } else {
          params.framerate = atoi(argv[i+1]);
        }
        break;
      }
      case 2:
      {
        if (use_default_tag) {
          params.color_mode = static_cast<ColorMode>(config[i].default_val);
        } else {
          params.color_mode = static_cast<ColorMode>(atoi(argv[i+1]));
        }
        break;
      }
      case 3:
      {
        if (use_default_tag) {
          params.depth_mode = static_cast<DepthMode>(config[i].default_val);
        } else {
          params.depth_mode = static_cast<DepthMode>(atoi(argv[i+1]));
        }
        break;
      }
      case 4:
      {
        if (use_default_tag) {
          params.stream_mode = static_cast<StreamMode>(config[i].default_val);
        } else {
          params.stream_mode = static_cast<StreamMode>(atoi(argv[i+1]));
        }
        break;
      }
      case 5:
      {
        if (use_default_tag) {
          params.color_stream_format =
            static_cast<StreamFormat>(config[i].default_val);
        } else {
          params.color_stream_format =
            static_cast<StreamFormat>(atoi(argv[i+1]));
        }
        break;
      }
      case 6:
      {
        if (use_default_tag) {
          params.depth_stream_format =
            static_cast<StreamFormat>(config[i].default_val);
        } else {
          params.depth_stream_format =
            static_cast<StreamFormat>(atoi(argv[i+1]));
        }
        break;
      }
      case 7:
      {
        if (use_default_tag) {
          params.state_ae = static_cast<bool>(config[i].default_val);
        } else {
          params.state_ae = static_cast<bool>(atoi(argv[i+1]));
        }
        break;
      }
      case 8:
      {
        if (use_default_tag) {
          params.state_awb = static_cast<bool>(config[i].default_val);
        } else {
          params.state_awb = static_cast<bool>(atoi(argv[i+1]));
        }
        break;
      }
      default:
        break;
    }
  }

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  util::Counter counter;
  for (;;) {
    counter.Update();
  }

  cam.Close();
  cv::destroyAllWindows();
  return 0;
}
