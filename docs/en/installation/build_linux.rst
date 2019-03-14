.. _build_linux:

Quick Start Guide for Linux
===========================

1. Install SDK dependencies
---------------------------

1.1 Install OpenCV
~~~~~~~~~~~~~~~~~~

*If you have installed opencv already or you want use it in ROS, you can
skip this part.*

1.1.1 Install OpenCV with apt or compile (Choose one)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1.1 Install OpenCV with apt (Recommend)
'''''''''''''''''''''''''''''''''''''''''''

.. code-block:: bash

   sudo apt-get install libopencv-dev

1.1.1.2 Install OpenCV by Compile
'''''''''''''''''''''''''''''''''

   To build and install Opencv, please refer to
   `Installation in Linux <https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html>`_

Alternatively, refer to the command below:

.. code-block:: bash

   [compiler] sudo apt-get install build-essential
   [required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
   [optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

.. code-block:: bash

   git clone https://github.com/opencv/opencv.git
   cd opencv/
   git checkout tags/3.4.0

   cd opencv/
   mkdir build
   cd build/

   cmake ..

   make -j4
   sudo make install

1.2 Install PCL for Point Cloud sample (Optional)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   sudo apt-get install libpcl-dev libproj-dev libopenni2-dev libopenni-dev

1.3 Link libGL.so for TX1/TX2 compile bug (Optional)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   sudo ln -sf /usr/lib/aarch64-linux-gnu/tegra/libGL.so /usr/lib/aarch64-linux-gnu/libGL.so

2. Build SDK
------------

.. code-block:: bash

   git clone https://github.com/slightech/MYNT-EYE-D-SDK.git
   cd MYNT-EYE-D-SDK

2.1 Init SDK
~~~~~~~~~~~~

.. note::
   Because of the problem of device permissions, you must reinsert
   the camera device after the command is executed and on the same
   computer, this operation only needs to be done once.

.. code-block:: bash

   make init

2.2 Compile SDK
~~~~~~~~~~~~~~~

.. code-block:: bash

   make all

3. Run Samples
--------------

.. note::
  Open the rectified image by default (Run vio need to raw image,
  run depth or points cloud need to rectified image.)

1) get_image shows the left camera image and colorful depthmap
   (compatible with USB2.0)

.. code-block:: bash

   ./samples/_output/bin/get_image

2) get_stereo_image shows the left camera image and colorful depthmap

.. code-block:: bash

   ./samples/_output/bin/get_stereo_image

3) get_depth shows the left camera image, 16UC1 depthmap and depth
   value(mm) on mouse pointed pixal

.. code-block:: bash

   ./samples/_output/bin/get_depth

4) get_points shows the left camera image, 16UC1 depthmap and point
   cloud view

.. code-block:: bash

   ./samples/_output/bin/get_points

5) get_imu shows motion datas

.. code-block:: bash

   ./samples/_output/bin/get_imu

6) get_img_params show camera intrinsics and save in file

.. code-block:: bash

   ./samples/_output/bin/get_img_params

7) get_imu_params show imu intrinsics and save in file

.. code-block:: bash

   ./samples/_output/bin/get_imu_params

8) get_from_callbacks show image and imu data by callback

.. code-block:: bash

   ./samples/_output/bin/get_from_callbacks

9) get_all_with_options open device with different options

.. code-block:: bash

   ./samples/_output/bin/get_all_with_options

4 Install With OpenCV ROS
-------------------------

If you won’t use ROS(The Robot Operating System), you can skip this
part.

4.1 Install ROS Kinetic
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   cd ~
   wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
   chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic

.. note::

   ROS Kinetic will install OpenCV, JPEG.

4.2 Build ROS Wrapper
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   make ros

**Core:**

.. code-block:: bash

   roscore

**RViz Display:**

.. code-block:: bash

   source ./wrappers/ros/devel/setup.bash
   roslaunch mynteye_wrapper_d display.launch

**Publish:**

.. code-block:: bash

   source ./wrappers/ros/devel/setup.bash
   roslaunch mynteye_wrapper_d mynteye.launch

4.3 Build Beta Device ROS Wrapper
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   make ros

**Core:**

.. code-block:: bash

   roscore

**RViz Display:**

.. code-block:: bash

   source ./wrappers/beta_ros/devel/setup.bash
   roslaunch mynteye_wrapper_d_beta display.launch

**Publish:**

.. code-block:: bash

   source ./wrappers/beta_ros/devel/setup.bash
   roslaunch mynteye_wrapper_d_beta mynteye.launch

**Subscribe:**

.. code-block:: bash

   source ./wrappers/beta_ros/devel/setup.bash
   rosrun mynteye_wrapper_d_beta mynteye_listener_d_beta

**Subscribe:**

.. code-block:: bash

   source ./wrappers/beta_ros/devel/setup.bash
   rosrun mynteye_wrapper_d_beta mynteye_listener_d_beta

5. Package
----------

If you wanna package with specified OpenCV version:

.. code-block:: bash

   cd <sdk>
   make cleanall
   export OpenCV_DIR=<install prefix>

   export OpenCV_DIR=/usr/local
   export OpenCV_DIR=$HOME/opencv-2.4.13.3

Packaging:

.. code-block:: bash

   cd <sdk>
   make pkg

6. Clean
--------

.. code-block:: bash

   cd <sdk>
   make cleanall
