.. _get_image:

Get camera image(Compatible with USB2.0)
===========================================

Compatible with USB2.0 ,change to the resolution and frame rate for
USB 2.0 automatically.Using the ``DeviceMode::DEVICE_COLOR`` function
of the API, you can get color image，or use ``DeviceMode::DEVICE_ALL``
to get color and depth image.

Using ``GetStreamData()`` to get your data.

Reference code snippet:

.. code-block:: c++

   // Device mode, default DEVICE_ALL
   //   DEVICE_COLOR: IMAGE_LEFT_COLOR y IMAGE_RIGHT_COLOR - IMAGE_DEPTH n
   //   DEVICE_DEPTH: IMAGE_LEFT_COLOR n IMAGE_RIGHT_COLOR n IMAGE_DEPTH y
   //   DEVICE_ALL:   IMAGE_LEFT_COLOR y IMAGE_RIGHT_COLOR - IMAGE_DEPTH y
   // Note: y: available, n: unavailable, -: depends on #stream_mode
   params.dev_mode = DeviceMode::DEVICE_DEPTH;

   auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
       if (left_color.img) {
       cv::Mat left = left_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
       painter.DrawSize(left, CVPainter::TOP_LEFT);
       painter.DrawStreamData(left, left_color, CVPainter::TOP_RIGHT);
       painter.DrawInformation(left, util::to_string(counter.fps()),
           CVPainter::BOTTOM_RIGHT);
       cv::imshow("left color", left);

Complete code samples，see
`get_image.cc <https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_image.cc>`__
.
