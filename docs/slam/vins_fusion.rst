.. _slam_vins_fusion:

How to use in `VINS-Fusion <https://github.com/HKUST-Aerial-Robotics/Vins-Fusion>`_
====================================================================================


If you wanna run VINS-Fusion with MYNT EYE camera, please follow the steps:
----------------------------------------------------------------------------

1. Download  `MYNT-EYE-D-SDK <https://github.com/slightech/MYNT-EYE-D-SDK.git>`_  and  :ref:`ros_install` .
2. Follow the normal procedure to install VINS-Fusion .
3. Run mynteye_wrapper_d and VINS-Fusion .


Install ROS Kinetic conveniently (if already installed, please ignore)
----------------------------------------------------------------------

.. code-block:: bash

  cd ~
  wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
  chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic

Run VINS-FUSION with docker
----------------------------

.. note::

  To complie with docker,we recommend that you should use more than 16G RAM, or ensure that the RAM and virtual memory space is greater than 16G.

Install docker
++++++++++++++++++++++

.. code-block:: bash

  sudo apt-get update
  sudo apt-get install \
      apt-transport-https \
      ca-certificates \
      curl \
      gnupg-agent \
      software-properties-common
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
  sudo add-apt-repository \
     "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
     $(lsb_release -cs) \
     stable"
  sudo apt-get update
  sudo apt-get install docker-ce docker-ce-cli containerd.io

.. tip::

  add your account to docker group by ``sudo usermod -aG docker $YOUR_USER_NAME`` . Relaunch the terminal or 
  logout and re-login if you get Permission denied error.

Install MYNT-EYE-VINS-FUSION-Samples
++++++++++++++++++++++++++++++++++++++

.. code-block::

  git clone -b docker_feat https://github.com/slightech/MYNT-EYE-VINS-FUSION-Samples.git
  cd MYNT-EYE-VINS-FUSION-Sample/docker
  make build

Run VINS_FUSION
+++++++++++++++++++++++

1. Run mynteye node

.. code-block:: bash

  cd MYNT-EYE-D-SDK (local path of MYNT-EYE-D-SDK)
  source ./wrappers/ros/devel/setup.bash
  roslaunch mynteye_wrapper_d vins_fusion.launch stream_mode:=1 # stereo camera with 640x480

2. Open another terminal to run vins-fusion

.. code-block:: bash

  cd MYNT-EYE-VINS-FUSION-Sample/docker (local path of MYNT-EYE-VINS-FUSION-Sample)
  ./run.sh mynteye-d/mynt_mono_config.yaml  # mono+imu fusion
  # ./run.sh mynteye-d/mynt_stereo_config.yaml  # Stereo fusion
  # ./run.sh mynteye-d/mynt_stereo_imu_config.yaml # Stereo+imu fusion
