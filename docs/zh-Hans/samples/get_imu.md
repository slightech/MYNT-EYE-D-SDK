# 获取IMU数据 {#get_imu}

使用``EnableMotionDatas()``来启用缓存，才能通过``GetMotionDatas()``函数来获取到IMU数据。否则，只能通过回调接口得到IMU数据，请参阅（从回调接口获取数据）[]。

参考代码片段：

```
auto motion_datas = cam.GetMotionDatas();
if (motion_datas.size() > 0) {
    std::cout << "Imu count: " << motion_datas.size() << std::endl;
    for (auto data : motion_datas) {
        if (data.imu) {
            if (data.imu->flag == MYNTEYE_IMU_ACCEL) {
                counter.IncrAccelCount();
                std::cout << "[accel] stamp: " << data.imu->timestamp
                    << ", x: " << data.imu->accel[0]
                    << ", y: " << data.imu->accel[1]
                    << ", z: " << data.imu->accel[2]
                    << ", temp: " << data.imu->temperature
                    << std::endl;
            } else if (data.imu->flag == MYNTEYE_IMU_GYRO) {
                counter.IncrGyroCount();
                std::cout << "[gyro] stamp: " << data.imu->timestamp
                << ", x: " << data.imu->gyro[0]
                << ", y: " << data.imu->gyro[1]
                << ", z: " << data.imu->gyro[2]
                << ", temp: " << data.imu->temperature
                << std::endl;
            } else {
                std::cerr << "Imu type is unknown" << std::endl;
            }
        } else {
          std::cerr << "Motion data is empty" << std::endl;
        }
    }
    std::cout << std::endl;
}
```

上述代码，用了 OpenCV 来显示图像和数据。选中显示窗口时，按 ESC/Q 就会结束程序。

完整代码样例，请见[get_imu.cc](https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_imu.cc) 。