
# MYNTEYE SDK

## Install

**ROS Kinetic:**

```
$ cd ~
$ wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic
```

**SDK:**

```
$ cd <sdk>
$ ./install.sh
```

## Samples

### Build

```
$ cd <sdk>
$ make samples
```

### Run

```
$ ./samples/build/output/bin/camera
```

## ROS Wrappers

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
$ rosrun mynteye_wrapper mynteye_listener
```

## Clean

```
$ cd <sdk>
$ make cleanall
```
