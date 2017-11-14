#ifndef MYNTEYE_API_CAMERA_H_
#define MYNTEYE_API_CAMERA_H_
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <opencv2/core/core.hpp>

#include "dev_info.h"
#include "init_params.h"
#include "mynteye.h"
#include "stream_info.h"

namespace mynteye {

class CameraPrivate;

class MYNTEYE_API Camera {
public:
    Camera();
    ~Camera();

    std::vector<DeviceInfo> GetDevices();
    void GetDevices(std::vector<DeviceInfo> &dev_infos);
    void GetResolutions(const std::int32_t &dev_index,
        std::vector<StreamInfo> &color_infos, std::vector<StreamInfo> &depth_infos);

    ErrorCode Open();
    ErrorCode Open(const InitParams &params);

    bool IsOpened();

    ErrorCode RetrieveImage(cv::Mat &color, cv::Mat &depth);
    //ErrorCode RetrieveImage(cv::Mat &mat, const View &view);

    void Close();

private:
    std::unique_ptr<CameraPrivate> d_ptr;

    friend class CameraPrivate;
};

}  // namespace mynteye

#endif  // MYNTEYE_API_CAMERA_H_
