.. _get_from_callbacks:

Get data from callbacks
=======================

API offers function ``SetStreamCallback()`` and ``SetMotionCallback()``
to set callbacks for various data.

Reference code snippet:

.. code-block:: c++

   cam.SetImgInfoCallback([](const std::shared_ptr<ImgInfo>& info) {
       std::cout << "  [img_info] fid: " << info->frame_id
           << ", stamp: " << info->timestamp
           << ", expos: " << info->exposure_time << std::endl
           << std::flush;
   });
   for (auto&& type : types) {
       // Set stream data callback
       cam.SetStreamCallback(type, [](const StreamData& data) {
       std::cout << "  [" << data.img->type() << "] fid: "
           << data.img->frame_id() << std::endl
           << std::flush;
       });
   }

   // Set motion data callback
   cam.SetMotionCallback([](const MotionData& data) {
       if (data.imu->flag == MYNTEYE_IMU_ACCEL) {
           std::cout << "[accel] stamp: " << data.imu->timestamp
           << ", x: " << data.imu->accel[0]
           << ", y: " << data.imu->accel[1]
           << ", z: " << data.imu->accel[2]
           << ", temp: " << data.imu->temperature
           << std::endl;
       } else if (data.imu->flag == MYNTEYE_IMU_GYRO) {
           std::cout << "[gyro] stamp: " << data.imu->timestamp
           << ", x: " << data.imu->gyro[0]
           << ", y: " << data.imu->gyro[1]
           << ", z: " << data.imu->gyro[2]
           << ", temp: " << data.imu->temperature
           << std::endl;
       }
       std::cout << std::flush;
   });

OpenCV is used to display images and data above. When the window is
selected, pressing ESC/Q will exit program.

Complete code examples, see
`get_from_callbacks.cc <https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_from_callbacks.cc>`__.
