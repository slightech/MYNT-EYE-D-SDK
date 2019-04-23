.. _slam_vins:

How to use in `VINS-Mono <https://github.com/HKUST-Aerial-Robotics/VINS-Mono>`_
================================================================================


If you wanna run VINS-Mono with MYNT EYE camera, please follow the steps:
--------------------------------------------------------------------------

1. Download `MYNT-EYE-D-SDK <https://github.com/slightech/MYNT-EYE-D-SDK.git>`__ and :ref:`ros_install`.
2. Follow the normal procedure to install VINS-Mono.
3. Run mynteye_wrapper_d and VINS-Mono.

Install ROS Kinetic conveniently (if already installed, please ignore)
----------------------------------------------------------------------

.. code-block:: bash

  cd ~
  wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
  chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic

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

Install MYNT-EYE-VINS-Sample
------------------------------

.. code-block:: bash

  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone https://github.com/slightech/MYNT-EYE-VINS-Sample.git
  cd ..
  catkin_make
  source devel/setup.bash
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc


Run VINS-Mono with MYNT® EYE
-----------------------------

1. Launch mynteye node

.. code-block:: bash

  cd (local path of MYNT-EYE-D-SDK)
  source ./wrappers/ros/devel/setup.bash
  roslaunch mynteye_wrapper_d vins_mono.launch

2. Open another terminal and run vins

.. code-block:: bash

  cd ~/catkin_ws
  roslaunch vins_estimator mynteye_d.launch

