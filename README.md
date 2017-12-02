
# MYNT EYE SDK - Depth Edition

> Only Linux x64 & aarch64 are supported.

> PC and TX2, with Ubuntu 16.04 (GCC 5), have been tested pass.

## Prerequisites

### With OpenCV Basic

**OpenCV, with same verion:**

```
$ git clone https://github.com/opencv/opencv.git
$ cd opencv/
$ git checkout tags/3.2.0

$ cd opencv/
$ mkdir build
$ cd build/

$ cmake \
-DCMAKE_BUILD_TYPE=RELEASE \
-DCMAKE_INSTALL_PREFIX=/usr/local \
\
-DWITH_CUDA=OFF \
\
-DBUILD_DOCS=OFF \
-DBUILD_EXAMPLES=OFF \
-DBUILD_TESTS=OFF \
-DBUILD_PERF_TESTS=OFF \
..

$ make -j
$ sudo make install
```

**JPEG:**

```
$ sudo apt-get install libjpeg-dev
```

### With OpenCV ROS

**ROS Kinetic:**

```
$ cd ~
$ wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic
```

> ROS Kinetic will install OpenCV, JPEG.

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

## Package

If wanna package with specified OpenCV version:

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

## Clean

```
$ cd <sdk>
$ make cleanall
```
