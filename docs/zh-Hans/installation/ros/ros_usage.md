# ROS 如何使用 {#ros_usage}

按照 \link ros_install ROS 安装 \endlink ，编译再运行节点。

`rostopic list` 可以列出发布的节点：

```bash
/mynteye/depth/camera_info
/mynteye/depth/image_raw
/mynteye/depth/image_raw/compressed
/mynteye/depth/image_raw/compressed/parameter_descriptions
/mynteye/depth/image_raw/compressed/parameter_updates
/mynteye/depth/image_raw/compressedDepth
/mynteye/depth/image_raw/compressedDepth/parameter_descriptions
/mynteye/depth/image_raw/compressedDepth/parameter_updates
/mynteye/depth/image_raw/theora
/mynteye/depth/image_raw/theora/parameter_descriptions
/mynteye/depth/image_raw/theora/parameter_updates
/mynteye/imu/data_raw
/mynteye/imu/data_raw_processed
/mynteye/left/camera_info
/mynteye/left/image_color
/mynteye/left/image_color/compressed
...
```

`rostopic hz <topic>` 可以检查是否有数据：

```bash
subscribed to [/mynteye/imu/data_raw]
average rate: 202.806
	min: 0.000s max: 0.021s std dev: 0.00819s window: 174
average rate: 201.167
	min: 0.000s max: 0.021s std dev: 0.00819s window: 374
average rate: 200.599
	min: 0.000s max: 0.021s std dev: 0.00819s window: 574
average rate: 200.461
	min: 0.000s max: 0.021s std dev: 0.00818s window: 774
average rate: 200.310
	min: 0.000s max: 0.021s std dev: 0.00818s window: 974
  ...
```

`rostopic echo <topic>` 可以打印发布数据等。了解更多，请阅读 [rostopic](http://wiki.ros.org/rostopic)。

ROS 封装的文件结构，如下所示：

```bash
<sdk>/wrappers/ros/
├─src/
  └─mynteye_wrapper_d/
     ├─launch/
     │  ├─display.launch
     │  └─mynteye.launch
     ├─msg/
     ├─rviz/
     ├─src/
     │  ├─mynteye_listener.cc
     │  └─mynteye_wrapper_nodelet.cc
     │  └─mynteye_wrapper_node.cc
     │  └─pointcloud_generatort.cc
     │  └─pointcloud_generator.h
     ├─CMakeLists.txt
     ├─nodelet_plugins.xml
     └─package.xml
```

其中 `mynteye.launch` 里，可以配置发布的 topics 与 frame_ids 、决定启用哪些数据、以及设定控制选项。其中，`gravity` 请配置成当地重力加速度。

```
<arg name="gravity" default="9.8" />
```