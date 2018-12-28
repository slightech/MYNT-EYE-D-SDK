# ROS 安装 {#ros_install}

### 4.1 安装 ROS Kinectic 版本

```
cd ~
wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic
```

> ROS Kinetic 会自动安装 OpenCV, JPEG.

### 4.2 编译 ROS Wrapper

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
roslaunch mynteye_wrapper_d display.launch
```

**Publish:**

```
source ./wrappers/ros/devel/setup.bash
roslaunch mynteye_wrapper_d mynteye.launch
```

**Subscribe:**

```
source ./wrappers/ros/devel/setup.bash
rosrun mynteye_wrapper_d mynteye_listener_d
```

### 4.3 编译内测版设备 ROS Wrapper

```
make ros
```

**Core:**

```
roscore
```

**RViz Display:**

```
source ./wrappers/beta_ros/devel/setup.bash
roslaunch mynteye_wrapper_d_beta display.launch
```

**Publish:**

```
source ./wrappers/beta_ros/devel/setup.bash
roslaunch mynteye_wrapper_d_beta mynteye.launch
```

**Subscribe:**

```
source ./wrappers/beta_ros/devel/setup.bash
rosrun mynteye_wrapper_d_beta mynteye_listener_d_beta
```
