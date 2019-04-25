.. _update_hid_firmware:

Update Auxiliary Chip
=========================

Get Auxiliary Chip Firmware
----------------------------

Latest firmware: mynteye-d-hid-firmware-1.2.bin `Google
Drive <https://drive.google.com/open?id=1gAbTf6W10a8iwT7L9TceMVgxQCWKnEsx>`__,
`Baidu Pan <https://pan.baidu.com/s/1sZKxugg5P8Dk5QgneA9ttw>`__

Compile SDK Tools
-----------------

.. code-block:: bash

   cd <sdk> #local path of MYNT-EYE-D-SDK
   make tools

Update Firmware
---------------

.. code-block:: bash

   ./tools/_output/bin/writer/device_hid_update <firmware-file-path>
