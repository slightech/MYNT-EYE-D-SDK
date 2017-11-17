#include <iomanip>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include "camera.h"

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

    InitParams params(dev_info.index);
    //params.depth_mode = DepthMode::DEPTH_GRAY;
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

    cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);

    double t, fps = 0;
    cv::Mat color, depth;
    for (;;) {
        t = (double)cv::getTickCount();

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

    cam.Close();
    cv::destroyAllWindows();
    return 0;
}
