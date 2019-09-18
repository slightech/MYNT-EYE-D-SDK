Get image data
======================

Set image information callback
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: kotlin

    mCamera?.setFrameCallback { data ->
        if (data.flag == FrameData.DEPTH) {
            // depth image
        }
        if (data.flag == FrameData.COLOR) {
            // color image
        }
    }