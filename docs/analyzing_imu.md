# Analyzing IMU
The SDK provides the script imu_analytics.py for IMU analysis. The tool details can be seen in tools/README.md .

Reference to run commands on Linux:

```
$ python tools/analytics/imu_analytics.py -i dataset -c tools/config/mynteye/mynteye_config.yaml -al=-1.2,1.2 -gl= -gdu=d -gsu=d -kl=
```
Note:: The analysis result graph will be saved in the data set directory.

In addition, the script specific options can be executed -h:

```
$ python tools/analytics/imu_analytics.py -h
```
