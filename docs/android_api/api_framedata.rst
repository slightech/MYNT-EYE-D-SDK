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

Image width
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public int width;

Image height
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public int height;

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

    public byte[] getDistanceShorts()


Get distance（only the flag is "DEPTH"）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public int getDistanceValue(int index)

Get distance（only the flag is "DEPTH"）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public int getDistanceValue(int x, int y)