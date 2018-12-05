# 从回调接口获取数据 {#get_from_callbacks}

API提供了``SetStreamCallback()``，``SetMotionCallback()``函数，来设定各类数据的回调。

参考代码片段：

```
cam.SetImgInfoCallback([](const std::shared_ptr<ImgInfo>& info) {
    std::cout << "  [img_info] fid: " << info->frame_id
        << ", stamp: " << info->timestamp
        << ", expos: " << info->exposure_time << std::endl
        << std::flush;
});
for (auto&& type : types) {
    // Set stream data callback
    cam.SetStreamCallback(type, [](const StreamData& data) {
    std::cout << "  [" << data.img->type() << "] fid: "
        << data.img->frame_id() << std::endl
        << std::flush;
    });
}

// Set motion data callback
cam.SetMotionCallback([](const MotionData& data) {
    if (data.imu->flag == MYNTEYE_IMU_ACCEL) {
        std::cout << "[accel] stamp: " << data.imu->timestamp
        << ", x: " << data.imu->accel[0]
        << ", y: " << data.imu->accel[1]
        << ", z: " << data.imu->accel[2]
        << ", temp: " << data.imu->temperature
        << std::endl;
    } else if (data.imu->flag == MYNTEYE_IMU_GYRO) {
        std::cout << "[gyro] stamp: " << data.imu->timestamp
        << ", x: " << data.imu->gyro[0]
        << ", y: " << data.imu->gyro[1]
        << ", z: " << data.imu->gyro[2]
        << ", temp: " << data.imu->temperature
        << std::endl;
    }
    std::cout << std::flush;
});
```

上述代码，用了 OpenCV 来显示图像和数据。选中显示窗口时，按 ESC/Q 就会结束程序。

完整代码样例，请见 [get_from_callbacks.cc](https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_from_callbacks.cc)。