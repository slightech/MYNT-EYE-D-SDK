.. _slam_orb_slam2:

How to use in `ORB_SLAM2 <https://github.com/raulmur/ORB_SLAM2>`_
==================================================================


If you wanna run ORB_SLAM2 with MYNT EYE camera, please follow the steps:
-------------------------------------------------------------------------

1. Download `MYNT-EYE-D-SDK <https://github.com/slightech/MYNT-EYE-D-SDK.git>`_ and :ref:`ros_install`.
2. Follow the normal procedure to install ORB_SLAM2.
3. Run examples by MYNT® EYE.

Prerequisites
------------------

.. code-block:: bash

    sudo apt-get -y install libglew-dev cmake
    cd ~
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build
    cd build
    cmake ..
    cmake --build .
    sudo make install

Building the nodes for stereo (ROS)
--------------------------------------------

* Add the path including ``Examples/ROS/ORB_SLAM2`` to the ``ROS_PACKAGE_PATH`` environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM2:

.. code-block:: bash

  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS

* Execute `build_ros.sh`:

.. code-block:: bash

  chmod +x build.sh
  ./build.sh
  chmod +x build_ros.sh
  ./build_ros.sh

Stereo_ROS Example
~~~~~~~~~~~~~~~~~~~

Run camera ``mynteye_wrapper_d``

.. code-block:: bash

  cd [path of mynteye-d-sdk]
  make ros
  source ./wrappers/ros/devel/setup.bash
  roslaunch mynteye_wrapper_d orb_slam2.launch

Open another terminal and run ORB_SLAM2

.. code-block:: bash

  rosrun ORB_SLAM2 mynteye_d_stereo ./Vocabulary/ORBvoc.txt ./config/mynteye_d_stereo.yaml true /mynteye/left/image_mono /mynteye/right/image_mono
