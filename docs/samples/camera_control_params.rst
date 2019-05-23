.. _camera_control_params:

Camera control parameters API
=============================

Open or close auto exposure
---------------------------

.. code-block:: c++

   /** Auto-exposure enabled or not  default enabled*/
   bool AutoExposureControl(bool enable);    see "camera.h"

Open or close auto white balance
--------------------------------

.. code-block:: c++

   /** Auto-white-balance enabled or not  default enabled*/
   bool AutoWhiteBalanceControl(bool enable);    see "camera.h"

Set infrared(IR) intensity
--------------------------

.. code-block:: c++

   /** set infrared(IR) intensity [0, 10] default 4*/
   void SetIRIntensity(const std::uint16_t &value);     see "camera.h"

Set global gain
---------------
.. note::
   You have to close auto exposure first after opening camera.

.. code-block:: c++

   /** Set global gain [1 - 16]
    * value -- global gain value
    * */
   void SetGlobalGain(const float &value);    see "camera.h"

Set the exposure time
---------------------

.. note::
 You have to close auto exposure first after opening camera.

.. code-block:: c++

   /** Set exposure time [1ms - 655ms]
    * value -- exposure time value
    * */
   void SetExposureTime(const float &value);    see "camera.h"

Reference code snippet:

.. code-block:: c++

  cam.Open(params);
  cam.AutoExposureControl(false);
  cam.SetGlobalGain(1);
  cam.SetExposureTime(0.3);

.. note::
  After changing the parameters, you need to run in the sdk directory

  .. code-block:: bash

    make samples

  to make the set parameters take effect.