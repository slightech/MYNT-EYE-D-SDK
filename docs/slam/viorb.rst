.. _slam_viorb:

How to use in `VIORB <https://github.com/jingpang/LearnVIORB>`_
================================================================


If you wanna run VIORB with MYNT® EYE，please follow the steps:
---------------------------------------------------------------

1. Download `MYNT-EYE-D-SDK <https://github.com/slightech/MYNT-EYE-D-SDK.git>`_ and :ref:`ros_install`.
2. Follow the normal procedure to install VIORB.
3. Update camera parameters to ``<VIO>/config/mynteye_d.yaml``.
4. Run mynteye_wrapper_d and VIORB.

Install MYNT-EYE-VIORB-Sample.
------------------------------

.. code-block:: bash

  git clone -b mynteye https://github.com/slightech/MYNT-EYE-VIORB-Sample.git
  cd MYNT-EYE-VIORB-Sample

``ROS_PACKAGE_PATH`` environment variable. Open ``.bashrc`` file and add at the end the following line. Replace ``PATH`` by the folder where you cloned ``MYNT-EYE-VIORB-Sample``:

.. code-block:: bash

  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/Examples/ROS/ORB_VIO

Execute:

.. code-block:: bash

  cd MYNT-EYE-VIORB-Sample
  ./build.sh

Get image calibration parameters
----------------------------------

Assume that the left eye of the mynteye camera is used with IMU. Through the ``GetIntrinsics()`` and ``GetExtrinsics()`` function of the `MYNT-EYE-D-SDK <https://github.com/slightech/MYNT-EYE-D-SDK.git>`_ API, you can get the image calibration parameters of the currently open device:

.. code-block:: bash

  cd MYNT-EYE-S-SDK
  ./samples/_output/bin/get_img_params

After running the above type, pinhole's ``distortion_parameters`` and ``projection_parameters`` is obtained, and then update to ``<MYNT-EYE-VIORB-Sample>/config/mynteye_d.yaml``.


Run VIORB and mynteye_wrapper_d
--------------------------------------

1. Launch mynteye node

.. code-block:: bash

  roslaunch mynteye_wrapper_d mynteye.launch

2. Open another terminal and run viorb

.. code-block:: bash

  roslaunch ORB_VIO testmynteye_d.launch

Finally, ``pyplotscripts`` can be used to visualize some results.

