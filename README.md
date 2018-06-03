
# MYNT EYE SDK - Depth Edition Quick Start Guide

> Only Linux x64 & aarch64 are supported.

> PC and TX2, with Ubuntu 16.04 (GCC 5), have been tested pass.

## Prerequisites

### 1. Install With OpenCV Basic

If you have installed opencv already or you want use it in ROS, you can skip this part.

**1.1 OpenCV with GTK or VTK**

```
$ git clone https://github.com/opencv/opencv.git
$ cd opencv/
$ git checkout tags/3.4.0

$ cd opencv/
$ mkdir build
$ cd build/

$ cmake ..

$ make -j4
$ sudo make install
```

**1.2 JPEG:**

```
$ sudo apt-get install libjpeg-dev
```
**1.3 Build:**

```
$ make all
```

**1.4 Samples**

```
$ ./samples/build/output/bin/camera
```

### 2. Install With OpenCV ROS

If you won't use ROS(The Robot Operating System), you can skip this part.

**2.1 Install ROS Kinetic:**

```
$ cd ~
$ wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic
```

> ROS Kinetic will install OpenCV, JPEG.

**2.2 Build ROS Wrapper**

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

## 3. Package

If you wanna package with specified OpenCV version:

```
$ make cleanall
$ export OpenCV_DIR=<install prefix>

$ export OpenCV_DIR=/usr/local
$ export OpenCV_DIR=$HOME/opencv-2.4.13.3
```

Packaging:

```
$ cd <sdk>
$ make pkg
```

## 4. Clean

```
$ cd <sdk>
$ make cleanall
```
