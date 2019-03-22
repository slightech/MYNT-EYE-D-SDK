.. _get_depth:

Get depth image
===============

Depth images belongs to the upper layer of synthetic data.

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
     cv::Mat depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();

     cv::setMouseCallback("depth", OnDepthMouseCallback, &depth_region);
     // Note: DrawRect will change some depth values to show the rect.
     depth_region.DrawRect(depth);
     cv::imshow("depth", depth);

     depth_region.ShowElems<ushort>(depth, [](const ushort& elem) {
       return std::to_string(elem);
     }, 80, depth_info);
   }

The above code uses OpenCV to display the image. When the display window
is selected, pressing ESC/Q will end the program.

.. note::

  `get_depth` sample only support  `DEPTH_RAW` mode.You can modify ``depth_mode`` parameter of other samples to get depth images 。

Complete code examples, see
`get_depth.cc <https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_depth.cc>`__.
