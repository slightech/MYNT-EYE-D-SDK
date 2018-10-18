# SDK Quick Start Guide for Linux

## 1. Install SDK dependencies

Note: Because of the problem of device permissions, you must reinsert the camera device after the command is executed.
```
make init
```

### 1.1 Install OpenCV

*If you have installed opencv already or you want use it in ROS, you can skip this part.*

#### 1.1.1 Install OpenCV with apt or compile (Choose one)

##### 1.1.1.1 Install OpenCV with apt (Recommond)

```
sudo apt-get install libopencv-dev
```

##### 1.1.1.2 Install OpenCV by Compile

```
git clone https://github.com/opencv/opencv.git
cd opencv/
git checkout tags/3.4.0

cd opencv/
mkdir build
cd build/

cmake ..

make -j4
sudo make install
```

### 1.2 Install PCL for Point Cloud sample (Optional)

```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```

### 1.3 Link libGL.so for TX1/TX2 compile bug (Optional)

```
sudo ln -sf /usr/lib/aarch64-linux-gnu/tegra/libGL.so /usr/lib/aarch64-linux-gnu/libGL.so
```

## 2. Build SDK

```
cd <sdk>
make all
```

## 3. Run Samples

1) get_image shows the left camera image and colorful depthmap

```
./samples/_output/bin/get_image
```

2) get_depth shows the left camera image, 16UC1 depthmap and depth value(mm) on mouse pointed pixal
```
./samples/_output/bin/get_depth
```

3) get_points shows the left camera image, 16UC1 depthmap and point cloud view
```
./samples/_output/bin/get_points
```

## 4 Install With OpenCV ROS

If you won't use ROS(The Robot Operating System), you can skip this part.

### 4.1 Install ROS Kinetic

```
cd ~
wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic
```

> ROS Kinetic will install OpenCV, JPEG.

### 4.2 Build ROS Wrapper

```
make ros
```

**Core:**

```
roscore
```

**RViz Display:**

```
source ./wrappers/ros/devel/setup.bash
roslaunch mynteye_wrapper display.launch
```

**Publish:**

```
source ./wrappers/ros/devel/setup.bash
roslaunch mynteye_wrapper mynteye.launch
```

**Subscribe:**

```
source ./wrappers/ros/devel/setup.bash
rosrun mynteye_wrapper mynteye_listener
```

## 5. Package

If you wanna package with specified OpenCV version:

```
cd <sdk>
make cleanall
export OpenCV_DIR=<install prefix>

export OpenCV_DIR=/usr/local
export OpenCV_DIR=$HOME/opencv-2.4.13.3
```

Packaging:

```
cd <sdk>
make pkg
```

## 6. Clean

```
cd <sdk>
make cleanall
```
