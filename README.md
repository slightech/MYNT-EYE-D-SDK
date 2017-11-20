
# Etron SDK

## Prerequisites

**Install ROS Kinetic:**

```
$ cd ~
$ wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic
```

* OpenCV (ROS Kinetic)
* JPEG

> Only Linux x64 & aarch64 are supported.

> PC and TX2, with Ubuntu 16.04 (GCC 5), have been tested pass.

## Build

```
$ make all
```

## Samples

```
$ ./samples/build/output/bin/camera
```

## ROS Wrapper

```
$ make ros
```

**Core:**

```
$ roscore
```

**RViz Display:**

```
$ source ./wrappers/ros/devel/setup.bash
$ roslaunch mynteye_wrapper display.launch
```

**Publish:**

```
$ source ./wrappers/ros/devel/setup.bash
$ roslaunch mynteye_wrapper mynteye.launch
```

**Subscribe:**

```
$ source ./wrappers/ros/devel/setup.bash
$ rosrun mynteye_wrapper mynteye_listener
```
