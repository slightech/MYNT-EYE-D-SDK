
# Etron SDK

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

**Publish:**

```
$ roscore
```

```
$ source ./wrappers/ros/devel/setup.bash

$ roslaunch mynteye_wrapper mynteye.launch
```

**Subscribe:**

```
$ source ./wrappers/ros/devel/setup.bash

$ rosrun mynteye_wrapper mynteye_listener
```
