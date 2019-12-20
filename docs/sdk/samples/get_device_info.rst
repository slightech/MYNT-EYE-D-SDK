.. _get_device_info:

Get Device Info
==============================

Use ``util::select()`` to get current
device Info.

Reference code snippet:

.. code-block:: c++

   DeviceInfo dev_info;
   if (!util::select(cam, &dev_info)) {
      return 1;
   }

Complete code examples, see
`get_device_info.cc <https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_device_info.cc>`__ .
