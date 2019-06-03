.. _change_log:

Change log
============

2019-05-29 v1.7.7
-------------------------

1. Add relink function.

2. Add ros wrapper independent compilations.


2019-04-26 v1.7.6
--------------------------

1. Fix ir_depth_only no depth image issue.

2. Fix point cloud jitter issue for ros display.


2019-04-17 v1.7.5
-------------------

1. Remove beta_ros wrapper.

2. Publish default camera info for beta device.

3. Add view point cloud ply file sample.

4. Add slam launch to ros wrapper.

5. Fix color anomaly issue for ros display.


2019-03-25 v1.7.4
-----------------

1. Fix compatibility problem of different devices in ros camera info.

2. Fix build problem when use specify opencv version under Ubuntu 18.


2019-03-18 v1.7.3
-----------------

1. Add support for external sensors (ultrasonic sensors, GPS).

2. Depth images and color images are synchronized by frame id.

3. Add sample which compatible with USB2.0.

4. Fix the problem that the frame rate of camera info released by left and right eyes under ROS is twice the normal value.

5. Document optimization.