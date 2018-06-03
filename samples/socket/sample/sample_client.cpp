#include "socket_client.hpp"

#include <memory> // unique_ptr
#include <cstdlib> // atoi

#include <iomanip>
#include <iostream>

#include "camera.h"

#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace mynteye;

void AssertCond(bool assert_cond, const char* fail_msg) {
  if (!assert_cond) {
    printf("Error: %s\nUsage: ./pic-client <serverip> <port> <rows> <cols>\n", fail_msg);
    exit(1);
  }
}

void ParseArgs(int argc, char** argv) {
  AssertCond(argc == 5, "Wrong number of arguments");
}

int main(int argc, char **argv) {
    ParseArgs(argc, argv);
    const char hostname[] = argv[1];
    int port = atoi(argv[2]);
    int cols = atoi(argv[3]);
    int rows = atoi(argv[4]);
    std::unique_ptr<SocketClient> client_ptr(new SocketClient(hostname, port));
    client_ptr->ConnectToServer();
    client_ptr->SendImageDims(cols, rows);

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

    cout << "\033[1;32mPress ESC/Q on Windows to terminate\033[0m" << endl;

    // cv::namedWindow("client_color", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("client_depth", cv::WINDOW_AUTOSIZE);

    double t, fps = 0;
    cv::Mat color, depth;
    for (;;) {
        t = (double)cv::getTickCount();

        if (cam.RetrieveImage(color, depth) == ErrorCode::SUCCESS) {
            // cv::imshow("client_color", color);
            // cv::imshow("client_depth", depth);
            client_ptr->SendImage(depth);
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
