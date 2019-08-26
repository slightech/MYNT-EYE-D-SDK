.. _get_image_params:

Get Image Calibration Parameters
================================

Use ``GetStreamIntrinsics()`` and ``GetStreamExtrinsics()`` to get image
calibration parameters.

Reference code snippet：

.. code-block:: c++

   auto vga_intrinsics = cam.GetStreamIntrinsics(StreamMode::STREAM_1280x480, &in_ok);
   auto vga_extrinsics = cam.GetStreamExtrinsics(StreamMode::STREAM_1280x480, &ex_ok);
   std::cout << "VGA Intrinsics left: {" << vga_intrinsics.left << "}" << std::endl;
   std::cout << "VGA Intrinsics right: {" << vga_intrinsics.right << "}" << std::endl;
   std::cout << "VGA Extrinsics left to right: {" << vga_extrinsics << "}" << std::endl;
   out << "VGA Intrinsics left: {" << vga_intrinsics.left << "}" << std::endl;
   out << "VGA Intrinsics right: {" << vga_intrinsics.right << "}" << std::endl;
   out << "VGA Extrinsics left to right: {" << vga_extrinsics << "}" << std::endl;

The result will be saved in the current file directory.Reference result
on Linux:

.. code-block:: bash

   VGA Intrinsics left: {width: [640], height: [480], fx: [358.45721435546875000], fy: [359.53115844726562500], cx: [311.12109375000000000], cy: [242.63494873046875000]coeffs: [-0.28297042846679688, 0.06178283691406250, -0.00030517578125000, 0.00218200683593750, 0.00000000000000000]}
   VGA Intrinsics right: {width: [640], height: [480], fx: [360.13885498046875000], fy: [360.89624023437500000], cx: [325.11029052734375000], cy: [251.46371459960937500]coeffs: [-0.30667877197265625, 0.08611679077148438, -0.00030136108398438, 0.00155639648437500, 0.00000000000000000]}
   VGA Extrinsics left to right: {rotation: [0.99996054172515869, 0.00149095058441162, 0.00875246524810791, -0.00148832798004150, 0.99999880790710449, -0.00030362606048584, -0.00875294208526611, 0.00029063224792480, 0.99996161460876465], translation: [-120.36341094970703125, 0.00000000000000000, 0.00000000000000000]}

.. note::

   In the parameters:
   Intrinsics provide values for ``fx`` , ``fy`` , ``cx`` , ``cy`` , then you can get intrinsic camera matrix (refer to
   `sensor_msgs/CameraInfo.msg <http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html>`__ ),
   distortion parameters ``coeffs`` contains values for ``k1`` , ``k2`` , ``p1`` , ``p2`` , ``k3`` .
   Extrinsics contains rotation matrix ``rotation`` , Translation matrix ``translation`` .


Complete code examples, see
`get_img_params.cc <https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_img_params.cc>`__ .
