# ROS installation

## 1 Install With OpenCV ROS

If you won't use ROS(The Robot Operating System), you can skip this part.

### 1.1 Install ROS Kinetic

```
cd ~
wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic
```

> ROS Kinetic will install OpenCV, JPEG.

### 1.2 Build ROS Wrapper

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
