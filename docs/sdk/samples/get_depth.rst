Get Depth Image(Compatible With USB2.0)
===============

In this sample you can learn how to display depth image and compute left
camera coordinate from depth image.

You can change ``depth_mode`` to change the display of the depth image.

.. code-block:: c++

   // Depth mode: colorful(default), gray, raw
   params.depth_mode = DepthMode::DEPTH_RAW;

Then you can get it through ``GetStreamData()``.In addition, it should
be check not be empty before use.

Reference code snippet:

.. code-block:: c++

    auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
    if (image_depth.img) {
      allow_count = true;
      auto &&depth_raw = image_depth.img->To(ImageFormat::DEPTH_RAW);
      auto &&depth_color =
          colorize->Process(depth_raw, ImageFormat::DEPTH_BGR)->ToMat();

      cv::setMouseCallback("depth", OnDepthMouseCallback, &depth_region);
      depth_region.DrawRect(depth_color);
      cv::imshow("depth", depth_color);

      // pass depth_raw to get real depth value
      depth_region.ShowElems<ushort>(
          depth_raw->ToMat(),
          [](const ushort& elem) {
            return std::to_string(elem);
          }, 80, depth_info);
    }

The above code uses OpenCV to display the image. When the display window
is selected, pressing ESC/Q will end the program.

.. note::

  You can use function ToMat() to convert raw depth frame to other format such
  as gray and colorful(raw:CV_16UC1, gray & colorful:CV).

Complete code examples, see
`get_depth.cc <https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_depth.cc>`__.
