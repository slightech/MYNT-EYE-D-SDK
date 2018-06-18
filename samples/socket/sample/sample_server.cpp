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
#include <memory> // unique_ptr
#include <sys/stat.h>

#include <opencv2/core/core.hpp> // Mat
#include <opencv2/highgui/highgui.hpp>

#include "socket_server.hpp"

bool DirExists(const char* path) {
  struct stat info;
  if (stat(path, &info) != 0) {
    perror("Can't access path");
    return 0;
  }
  else if(info.st_mode & S_IFDIR) {
    return 1;
  }
  else return 0;
}

void AssertCond(bool assert_cond, const char* fail_msg) {
  if (!assert_cond) {
    printf("Error: %s\nUsage: ./pic-server <port>\n", fail_msg);
    exit(1);
  }
}

void ParseArgs(int argc, char** argv) {
  AssertCond(argc == 2, "Wrong number of arguments");
}

int main(int argc, char** argv) {
  ParseArgs(argc, argv);
  int port = atoi(argv[1]);
  std::unique_ptr<SocketServer> server_ptr(new SocketServer(port));
  server_ptr->ConnectToNetwork();
  server_ptr->ReceiveImageDims();
  cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
  while(1) {
    cv::Mat image;
    server_ptr->ReceiveImage(image);
    cv::imshow("depth", image);

    char key = (char)cv::waitKey(10);
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
        break;
    }
  }
  return 1; // Should not return
}
