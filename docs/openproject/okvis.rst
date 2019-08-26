.. _okvis:

How To Use In `OKVIS <https://github.com/ethz-asl/okvis>`_
=============================================================

If you wanna run OKVIS with MYNT EYE camera, please follow the steps:
----------------------------------------------------------------------

1. Download `MYNT-EYE-D-SDK <https://github.com/slightech/MYNT-EYE-D-SDK.git>`_ and :ref:`install_ros`.
2. Install dependencies and build MYNT-EYE-OKVIS-Sample follow the procedure of the original OKVIS.
3. Update camera parameters to ``<OKVIS>/config/config_mynteye.yaml``.
4. Run OKVIS using MYNT® EYE.


.. tip::

    OKVIS doesn’t support ARM right now.

Install MYNTEYE OKVIS
---------------------

First install dependencies based on the original OKVIS, and the follow:

.. code-block:: bash

  sudo apt-get install libgoogle-glog-dev

  git clone -b mynteye https://github.com/slightech/MYNT-EYE-OKVIS-Sample.git
  cd MYNT-EYE-OKVIS-Sample/
  mkdir build && cd build
  cmake ..
  make

Get camera calibration parameters
----------------------------------

Through the ``GetIntrinsics()`` and ``GetExtrinsics()`` function of the `MYNT-EYE-D-SDK <https://github.com/slightech/MYNT-EYE-D-SDK.git>`_ API, you can get the camera calibration parameters of the currently open device, follow the steps:

.. code-block:: bat

  cd MYNT-EYE-D-SDK
  ./samples/_output/bin/get_img_params

After running the above type,  pinhole's ``distortion_parameters`` and ``camera parameters`` is obtained, and then update to `here <https://github.com/slightech/MYNT-EYE-OKVIS-Sample/blob/mynteye/config/config_mynteye_d.yaml>`_ .

according to following format. It should be noted that only first four parameters of coeffs need to be filled in the distortion_coefficients.

.. code-block:: bash

  distortion_coefficients: [coeffs]   # only first four parameters of coeffs need to be filled
  focal_length: [fx, fy]
  principal_point: [cx, cy]
  distortion_type: radialtangential

Run MYNTEYE OKVIS
---------------------

Run camera ``mynteye_wrapper_d``

.. code-block:: bash

   cd MYNT-EYE-D-SDK
   source wrappers/ros/devel/setup.bash
   roslaunch mynteye_wrapper_d mynteye.launch

Run ``MYNT-EYE-OKVIS-Sample`` open another terminal and follow the steps.

.. code-block:: bash

  cd MYNT-EYE-OKVIS-Sample/build
  source devel/setup.bash
  roslaunch okvis_ros mynteye_d.launch

And use rviz to display

.. code-block:: bash

  cd ~/catkin_okvis/src/MYNT-EYE-OKVIS-Sample/config
  rosrun rviz rviz -d rviz.rvi

