.. _ros_usage:

ROS Usage
=========

Compile and run the node according to :ref:`ros_install`.

``rostopic list`` lists all released nodes:

.. code:: bash

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

``rostopic hz <topic>`` checks the data:

.. code:: bash

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

``rostopic echo <topic>`` can print and release data. Please read
`rostopic <http://wiki.ros.org/rostopic>`__ for more information.

The ROS file is structured like follows:

.. code:: bash

   <sdk>/wrappers/ros/
   ├─src/
     └─mynteye_wrapper_d/
        ├─launch/
        │  ├─display.launch
        │  └─mynteye.launch
        │  └─slam
        │     ├─orb_slam2.launch
        │     └─vins_fusion.launch
        │     └─vins_mono.launch
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

In ``mynteye.launch`` ,you can configure ``topics`` and ``frame_ids``
,decide which data to enable, and set the control options.Please refer
 to :ref:`support_resolutions` to set frame rate and resolution. Please set
``gravity`` to the local gravity acceleration.

.. code-block:: c++

  <!-- Camera Params -->

  <!-- Device index -->
  <arg name="dev_index" default="0" />
  <!-- Framerate -->
  <arg name="framerate" default="30" />

  <!--
  Device mode
    device_color: left_color ✓ right_color ? depth x
    device_depth: left_color x right_color x depth ✓
    device_all:   left_color ✓ right_color ? depth ✓
  Note: ✓: available, x: unavailable, ?: depends on #stream_mode
  -->
  <arg name="dev_mode" default="$(arg device_all)" />

  <!-- 
  Set depth mode
  Note: must set DEPTH_RAW to get raw depth values for points
  -->
  <arg name="depth_mode" default="$(arg depth_raw)" />
  <!--
  Set resolution from stream_640x480,stream_1280x720,stream_1280x480,stream_2560x720
  -->
  <arg name="stream_mode" default="$(arg stream_2560x720)" />

  <!-- Auto-exposure -->
  <arg name="state_ae" default="true" />
  <!-- Auto-white balance -->
  <arg name="state_awb" default="true" />
  <!-- IR intensity -->
  <arg name="ir_intensity" default="4" />
  <!-- IR Depth Only -->
  <arg name="ir_depth_only" default="false" />

  <!-- Setup your local gravity here -->
  <arg name="gravity" default="9.8" />
