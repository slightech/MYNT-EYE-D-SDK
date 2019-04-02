.. _ros_install:

ROS Installation
================

1 Install With OpenCV ROS
-------------------------

If you won't use ROS(The Robot Operating System), you can skip this
part.

1.1 Install ROS Kinetic
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   cd ~
   wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
   chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic

..

   ROS Kinetic will install OpenCV, JPEG.

1.2 Build ROS Wrapper
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

**Subscribe:**

.. code-block:: bash

   source ./wrappers/ros/devel/setup.bash
   rosrun mynteye_wrapper_d mynteye_listener_d

1.3 Build Beta Device ROS Wrapper
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
