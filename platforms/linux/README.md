
# MYNT EYE SDK - Depth Edition

## Prerequisites

### OpenCV Basic Packages

> mynteye-\*-opencv-<version, such as 3.2.0>.tar.gz

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

### OpenCV ROS Packages

> mynteye-\*-opencv-ros-kinetic.tar.gz

**ROS Kinetic:**

```
$ cd ~
$ wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic
```

## Install SDK

Edit `<sdk>/sdk.cfg` to set your OpenCV directory etc., then install:

```
$ cd <sdk>
$ ./install.sh
```

## The Samples

### Build

```
$ cd <sdk>
$ make samples
```

### Run

```
$ ./samples/_output/bin/get_image
```

## The ROS Wrapper

### Build

```
$ cd <sdk>
$ make ros
```

### Run

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
$ rosrun mynteye_wrapper mynteye_listener_d
```

## Clean All

```
$ cd <sdk>
$ make cleanall
```
