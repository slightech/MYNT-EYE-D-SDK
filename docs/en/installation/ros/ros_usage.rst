.. role:: raw-latex(raw)
   :format: latex
..

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
,decide which data to enable, and set the control options. Please set
``gravity`` to the local gravity acceleration.

.. code-block:: c++

   <arg name="gravity" default="9.8" />
