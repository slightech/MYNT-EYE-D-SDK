ImuData （Device with Imu ）
============================================

Data Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    /**
     * Data type
     *
     * 1: accelerometer
     * 2: gyroscope
     *
     * */
    public int flag;

Timestamp
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public long timestamp;

Temperature
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    public double temperature;

Data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: java

    /**
     * Imu accelerometer data for 3-axis: X, Y, X.
     * Imu gyroscope data for 3-axis: X, Y, Z.
     *
     * */
    public double value[];