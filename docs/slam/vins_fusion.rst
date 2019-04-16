.. _slam_vins_fusion:

How to use in `VINS-Fusion <https://github.com/HKUST-Aerial-Robotics/Vins-Fusion>`_
====================================================================================


If you wanna run VINS-Fusion with MYNT EYE camera, please follow the steps:
----------------------------------------------------------------------------

1. Download  `MYNT-EYE-D-SDK <https://github.com/slightech/MYNT-EYE-D-SDK.git>`_  and :ref:`ros_install`.
2. Follow the normal procedure to install VINS-Fusion .
3. Run mynteye_wrapper_d and VINS-Fusion .


Preparation
------------

1. Install Ubuntu 64bit 16.04/18.04. ROS Kinetic/Melodic(If you have installed ROS, you can skip this part).
`ROS Installation <http://wiki.ros.org/ROS/Installation>`_
2. Install `Ceres Installation <http://ceres-solver.org/installation.html>`_

Install Ceres
---------------

.. code-block:: bash

    cd ~
    git clone https://ceres-solver.googlesource.com/ceres-solver
    sudo apt-get -y install cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
    sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
    sudo apt-get update && sudo apt-get install libsuitesparse-dev
    mkdir ceres-bin
    cd ceres-bin
    cmake ../ceres-solver
    make -j3
    sudo make install

Build VINS-Fusion
-------------------

Clone the repository and catkin_make:

.. code-block:: bash

  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone -b mynteye https://github.com/slightech/MYNT-EYE-VINS-FUSION-Samples.git
  cd ..
  catkin_make
  source ~/catkin_ws/devel/setup.bash

(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

Run MYNTEYE VINS-Fusion
-------------------------

1.Launch mynteye node

.. code-block:: bash

  cd MYNT-EYE-D-SDK (local path of MYNT-EYE-D-SDK)
  source ./wrappers/ros/devel/setup.bash
  roslaunch mynteye_wrapper_d vins_fusion.launch

2.Open another terminal and run vins-fusion

.. code-block:: bash

  cd ~/catkin_ws
  roslaunch vins mynteye-d-mono-imu.launch  # mono+imu fusion
  # roslaunch vins mynteye-d-stereo.launch  # Stereo fusion / Stereo+imu fusion