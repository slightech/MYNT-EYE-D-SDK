.. _get_points:

Get point image
===============

Point images belongs to upper layer of synthetic data.You can get it
through ``GetStreamData()``.It should be check not empty before use.
Otherwise, when running pionts,you can use "space" to save ``.ply`` files.
Then sample ``view_points`` can be used to view ``.ply`` files.

Sample code snippet:

.. code-block:: c++

   auto image_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
   auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
   if (image_color.img && image_depth.img) {
       cv::Mat color = image_color.img->To(ImageFormat::COLOR_BGR)
           ->ToMat();
       painter.DrawSize(color, CVPainter::TOP_LEFT);
       painter.DrawStreamData(color, image_color, CVPainter::TOP_RIGHT);
       painter.DrawInformation(color, util::to_string(counter.fps()),
           CVPainter::BOTTOM_RIGHT);

       cv::Mat depth = image_depth.img->To(ImageFormat::DEPTH_RAW)
           ->ToMat();

       cv::imshow("color", color);

       viewer.Update(color, depth);
   }

PCL is used to display point images above. Program will close when point
image window is closed.

Complete code examples, see
`get_points.cc <https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_points.cc>`__.
