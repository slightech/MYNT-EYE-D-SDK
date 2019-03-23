.. _save_all_infos:

Save device infomation and parameters
=====================================

The SDK provides a tool ``save_all_infos`` for save information and
parameters.

Reference commands:

.. code-block:: bash

   ./tools/_output/bin/writer/save_all_infos

.. code-block:: bat

   # Windows
   .\tools\_output\bin\writer\save_all_infos.bat

Reference result on Linux:

.. code-block:: bash

   I/eSPDI_API: eSPDI: EtronDI_Init
   Device descriptors:
     name: MYNT-EYE-D1000
     serial_number: 203837533548500F002F0028
     firmware_version: 1.0
     hardware_version: 2.0
     spec_version: 1.0
     lens_type: 0000
     imu_type: 0000
     nominal_baseline: 120

Result save into ``<workdir>/config`` by default. You can also add
parameters to select other directory for save.

Saved contents:

::

   <workdir>/
   └─config/
      └─SN0610243700090720/
         ├─device.info
         └─imu.params

Complete code samples，see
`save_all_infos.cc <https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/tools/writer/save_all_infos.cc>`__
.
