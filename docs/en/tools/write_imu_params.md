# Write IMU parameters {#write_imu_params}

SDK provides the tool `imu_params_writer` to write IMU parameters.

Information about how to get IMU parameters, please read [Get IMU calibration parameters]() .

Reference commands:

```bash
./tools/_output/bin/writer/imu_params_writer tools/writer/config/imu.params

# Windows
.\tools\_output\bin\writer\imu_params_writer.bat tools\writer\config\imu.params
```

The path of parameters file can be found in [tools/writer/config/imu.params](https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/tools/writer/config/imu.params) . If you calibrated the parameters yourself, you can edit the file and run above commands to write them into the device.

> Warning
> - Please don’t override parameters, you can use `save_all_infos` to backup parameters.

Complete code samples，see [imu_params_writer](https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/tools/writer/imu_params_writer.cc) .
