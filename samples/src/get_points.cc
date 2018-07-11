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

// PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "camera.h"

using namespace std;
using namespace mynteye;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr cloud ( new PointCloud );
pcl::visualization::CloudViewer viewer("pcd viewer");  

// TODO replace the real camera param
const float camera_factor = 1000.0;

const double camera_cx = 682.3;
const double camera_cy = 254.9;
const double camera_fx = 979.8;
const double camera_fy = 942.8;

// show point cloud
void show_points(cv::Mat rgb, cv::Mat depth) {
    // loop the mat
    for (int m = 0; m < depth.rows; m++) {
        for (int n = 0; n < depth.cols; n++) {
            // get depth value at (m, n)
            unsigned short d = depth.ptr<unsigned short>(m)[n];
            // when d is equal 0 or 4096 means no depth
            if (d == 0 || d == 4096)
                continue;

            PointT p;

            // get point x y z
            p.z = float(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;
            
            // get colors
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            // add point to cloud
            cloud->points.push_back( p );
        }
    }

    viewer.showCloud(cloud);
    // clear points
    cloud->points.clear();
}

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
            if (i < 0 || i >= n) {
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
    params.depth_mode = DepthMode::DEPTH_NON_16UC1;
    //params.color_info_index = 4;
    //params.depth_info_index = 1;
    params.ir_intensity = 4;

    cam.Open(params);

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
    for (;;) {
        t = (double)cv::getTickCount();

        if (cam.RetrieveImage(color, depth) == ErrorCode::SUCCESS) {
            cv::imshow("color", color);
            cv::imshow("depth", depth);
            show_points(color, depth);
        }

        char key = (char)cv::waitKey(10);
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }

        t = (double)cv::getTickCount() - t;
        fps = cv::getTickFrequency() / t;
    }
    (void)(fps);

    cam.Close();
    cv::destroyAllWindows();
    return 0;
}
