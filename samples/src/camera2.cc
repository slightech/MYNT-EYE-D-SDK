#include <iomanip>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camera.h"

#define WIN_FLAGS cv::WINDOW_AUTOSIZE

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
    void OnMouse(const int &event, const int &x, const int &y, const int &flags) {
        if (event != CV_EVENT_MOUSEMOVE && event != CV_EVENT_LBUTTONDOWN) {
            return;
        }
        show_ = true;

        if (event == CV_EVENT_MOUSEMOVE) {
            if (!selected_) {
                point_.x = x;
                point_.y = y;
            }
        } else if (event == CV_EVENT_LBUTTONDOWN) {
            if (selected_) {
                if (x >= static_cast<int>(point_.x - n_) && x <= static_cast<int>(point_.x + n_) &&
                    y >= static_cast<int>(point_.y - n_) && y <= static_cast<int>(point_.y + n_)) {
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
    void ShowElems(const cv::Mat &depth,
            std::function<std::string(const T &elem)> elem2string,
            int elem_space = 40,
            std::function<std::string(const cv::Mat &depth, const cv::Point &point, const std::uint32_t &n)> getinfo = nullptr) {
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
        n += 1; // outside the region
#ifdef USE_OPENCV2
        cv::rectangle(const_cast<cv::Mat&>(im),
#else
        cv::rectangle(im,
#endif
            cv::Point(point_.x-n, point_.y-n),
            cv::Point(point_.x+n, point_.y+n),
            selected_ ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255), 1);
    }

private:
    std::uint32_t n_;
    bool show_;
    bool selected_;
    cv::Point point_;
};

void OnDepthMouseCallback(int event, int x, int y, int flags, void *userdata) {
    DepthRegion *region = reinterpret_cast<DepthRegion*>(userdata);
    region->OnMouse(event, x, y, flags);
}

}  // namespace

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

    cv::namedWindow("color", WIN_FLAGS);
    cv::namedWindow("depth", WIN_FLAGS);
    cv::namedWindow("region", WIN_FLAGS);

    DepthRegion depth_region(3);
    auto depth_info = [](const cv::Mat &depth, const cv::Point &point, const std::uint32_t &n) {
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

    double t, fps = 0;
    cv::Mat color, depth;
    for (;;) {
        t = (double)cv::getTickCount();

        if (cam.RetrieveImage(color, depth) == ErrorCode::SUCCESS) {
            cv::setMouseCallback("color", OnDepthMouseCallback, &depth_region);
            depth_region.DrawRect(color);
            cv::imshow("color", color);

            cv::setMouseCallback("depth", OnDepthMouseCallback, &depth_region);
            // Note: DrawRect will change some depth values to show the rect.
            depth_region.DrawRect(depth);
            cv::imshow("depth", depth);

            depth_region.ShowElems<ushort>(depth, [](const ushort &elem) {
                return std::to_string(elem);
            }, 80, depth_info);
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
