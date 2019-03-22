.. _get_imu:

Get IMU data
============

You need ``EnableMotionDatas()``\ to enable caching in order to get IMU
data from ``GetMotionDatas()``.Otherwise, IMU data is only available
through the callback interface, see :ref:`get_from_callbacks`.

Sample code snippet:

.. code-block:: c++

   auto motion_datas = cam.GetMotionDatas();
   if (motion_datas.size() > 0) {
       std::cout << "Imu count: " << motion_datas.size() << std::endl;
       for (auto data : motion_datas) {
           if (data.imu) {
               if (data.imu->flag == MYNTEYE_IMU_ACCEL) {
                   counter.IncrAccelCount();
                   std::cout << "[accel] stamp: " << data.imu->timestamp
                       << ", x: " << data.imu->accel[0]
                       << ", y: " << data.imu->accel[1]
                       << ", z: " << data.imu->accel[2]
                       << ", temp: " << data.imu->temperature
                       << std::endl;
               } else if (data.imu->flag == MYNTEYE_IMU_GYRO) {
                   counter.IncrGyroCount();
                   std::cout << "[gyro] stamp: " << data.imu->timestamp
                   << ", x: " << data.imu->gyro[0]
                   << ", y: " << data.imu->gyro[1]
                   << ", z: " << data.imu->gyro[2]
                   << ", temp: " << data.imu->temperature
                   << std::endl;
               } else {
                   std::cerr << "Imu type is unknown" << std::endl;
               }
           } else {
             std::cerr << "Motion data is empty" << std::endl;
           }
       }
       std::cout << std::endl;
   }

OpenCV is used to display image and data. When window is selected, press
ESC/Q to exit program.

Complete code examples, see
`get_imu.cc <https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_imu.cc>`__.
