.. _slam_vins:

How to use in `VINS-Mono <https://github.com/HKUST-Aerial-Robotics/VINS-Mono>`_
================================================================================


If you wanna run VINS-Mono with MYNT EYE camera, please follow the steps:
--------------------------------------------------------------------------

1. Download `MYNT-EYE-D-SDK <https://github.com/slightech/MYNT-EYE-D-SDK.git>`_ and :ref:`ros_install`.
2. Follow the normal procedure to install VINS-Mono.
3. Update ``distortion_parameters`` and ``projection_parameters`` to `here <https://github.com/slightech/MYNT-EYE-VINS-Sample/blob/mynteye/config/mynteye/mynteye_d_config.yaml>`_ .
4. Run mynteye_wrapper_d and VINS-Mono.

Install ROS Kinetic conveniently (if already installed, please ignore)
----------------------------------------------------------------------

.. code-block:: bash

  cd ~
  wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
  chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic

Install MYNT-EYE-VINS-Sample
------------------------------

.. code-block:: bash

  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone -b mynteye https://github.com/slightech/MYNT-EYE-VINS-Sample.git
  cd ..
  catkin_make
  source devel/setup.bash
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc

Get image calibration parameters
---------------------------------

Use MYNT® EYE's left eye camera and IMU. By `MYNT-EYE-D-SDK <https://github.com/slightech/MYNT-EYE-D-SDK.git>`_ API ``GetIntrinsics()`` function and ``GetExtrinsics()`` function, you can "get the image calibration parameters of the current working device:

.. code-block:: bash

  cd MYNT-EYE-D-SDK
  ./samples/_output/bin/get_img_params

After running the above type, pinhole's ``distortion_parameters`` and ``projection_parameters`` is obtained , and then update to `here <https://github.com/slightech/MYNT-EYE-VINS-Sample/blob/mynteye/config/mynteye/mynteye_d_config.yaml>`_ .


Run VINS-Mono with MYNT® EYE
-----------------------------

1. Launch mynteye node

.. code-block:: bash

  cd (local path of MYNT-EYE-D-SDK)
  source ./wrappers/ros/devel/setup.bash
  roslaunch mynteye_wrapper_d mynteye.launch

2. Open another terminal and run vins

.. code-block:: bash

  cd ~/catkin_ws
  roslaunch vins_estimator mynteye_d.launch

