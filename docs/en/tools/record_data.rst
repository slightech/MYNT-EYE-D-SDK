.. _record_data:

Record data sets
================

The SDK provides the tool record for recording data sets. Tool details
can be seen in tools/README.md .

Reference run command on Linux:

.. code-block:: bash

   ./tools/_output/bin/dataset/record

Reference run command on Windows:

.. code-block:: bash

   .\tools\_output\bin\dataset\record.bat

Reference run results on Linux:

.. code-block:: bash

   $ ./tools/_output/bin/dataset/record
   Saved 1007 imgs, 20040 imus to ./dataset
   I0513 21:29:38.608772 11487 record.cc:118] Time beg: 2018-05-13 21:28:58.255395, end: 2018-05-13 21:29:38.578696, cost: 40323.3ms
   I0513 21:29:38.608853 11487 record.cc:121] Img count: 1007, fps: 24.9732
   I0513 21:29:38.608873 11487 record.cc:123] Imu count: 20040, hz: 496.983

Results save into ``<workdir>/dataset`` by default. You can also add
parameter, select other directory to save.

Record contents:

::

   <workdir>/
   └─dataset/
      ├─left/
      │  ├─stream.txt  # Image infomation
      │  └─...
      ├─right/
      │  ├─stream.txt  # Image information
      │  └─...
      └─motion.txt  # IMU information

.. tip::

  When recording data, ``dataset.cc`` has annotated display image inside ``cv::imwrite()`` . Because these operations are time-consuming, they can cause images to be discarded. In other words, consumption can't keep up with production, so some images are discarded. ``GetStreamDatas()`` used in ``record.cc`` only caches the latest 4 images.


