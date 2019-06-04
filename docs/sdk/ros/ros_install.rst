.. _ros_install:

ROS Installation
================

1.1 Install With OpenCV ROS
-------------------------

If you won't use ROS(The Robot Operating System), you can skip this
part.


ROS Melodic (Ubuntu 18.04)
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  sudo apt update
  sudo apt install ros-melodic-desktop-full
  sudo rosdep init
  rosdep update

ROS Kinetic (Ubuntu 16.04)
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
  chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic

1.2 Build ROS Wrapper
--------------------------

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

**Subscribe:**

.. code-block:: bash

   source ./wrappers/ros/devel/setup.bash
   rosrun mynteye_wrapper_d mynteye_listener_d

