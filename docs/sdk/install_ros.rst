.. _install_ros:

ROS Wrapper Installation
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
  echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  source ~/.bashrc


ROS Kinetic (Ubuntu 16.04)
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  sudo apt-get update
  sudo apt-get install ros-kinetic-desktop-full
  sudo rosdep init
  rosdep update
  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc

1.2 Build ROS Wrapper
--------------------------

.. code-block:: bash

   make init
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


