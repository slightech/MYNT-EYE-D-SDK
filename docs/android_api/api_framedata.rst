FrameData
======================

Data type
~~~~~~~~~~

.. code:: java

    /**
     *
     * FrameData.COLOR
     * FrameData.DEPTH
     *
     * */
    public int flag;

TimeStamp
~~~~~~~~~~

.. code:: java

    public int frameId;

Image width
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public int width;

Image height
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public int height;

Color frame type（Left / Left && Right）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public ColorFrame colorMode;

Frame format
~~~~~~~~~~~~~~~~~~~~

.. code:: java
    
    /**
     * MYNTCamera.FRAME_FORMAT_YUYV
     * MYNTCamera.FRAME_FORMAT_MJPEG
     * MYNTCamera.PIXEL_FORMAT_RGBX
     *
     * */
    public int type;

Depth image type
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    /**
     * MYNTCamera.DEPTH_DATA_11_BITS
     * MYNTCamera.DEPTH_DATA_8_BITS
     *
     * */
    public int depthType;

Get bitmap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public Bitmap convert2Bitmap(byte[] bytes, int width, int height)

Get data from left camera
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public byte[] getLeftBytes()

Get data from right camera
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public byte[] getRightBytes()


Get distance array（only the flag is "DEPTH"）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public int[] getDistanceInts()

Get distance array（only the flag is "DEPTH"）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    /**
     * get distance table（int）
     *
     * @param max   Max(mm), if more than  max, go to be 0.
     *
     * */
    public int[] getDistanceInts(int max)
    
Get distance array（only the flag is "DEPTH"）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    /**
     * get distance table（int）
     *
     * @param min   Min(mm)
     * @param max   Max(mm), if more than  max, go to be 0.
     *
     * */
    public int[] getDistanceInts(int min, int max)

Get distance array（only the flag is "DEPTH"）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public byte[] getDistanceShorts()

Get distance array（only the flag is "DEPTH"）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    /**
     * get distance table（int）
     *
     * @param max   Max(mm), if more than  max, go to be 0.
     *
     * */
    public byte[] getDistanceShorts(int max)

Get distance array（only the flag is "DEPTH"）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    /**
     * get distance table（int）
     *
     * @param min   Min(mm)
     * @param max   Max(mm), if more than  max, go to be 0.
     *
     * */
    public byte[] getDistanceShorts(int min, int max)


Get distance（only the flag is "DEPTH"）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public int getDistanceValue(int index)

Get distance（only the flag is "DEPTH"）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public int getDistanceValue(int x, int y)