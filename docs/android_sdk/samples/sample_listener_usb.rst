Listener USB
======================

Initialize USB Monitor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: kotlin

   mUSBMonitor = USBMonitor(mContext, object : USBMonitor.IUSBMonitorListener {

       override fun didAttach(camera: MYNTCamera) {
           // Insert equipment
       }

       override fun didDettach(camera: MYNTCamera) {
           // Pull out equipment
       }

       override fun didConnectedCamera(camera: MYNTCamera) {
           // Connection successful
       }

       override fun didDisconnectedCamera(camera: MYNTCamera) {
           // Disconnect equipment
       }

   })

Register USB Monitor (start listening USB)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: kotlin

   mUSBMonitor?.register()

Log out of USB Monitor (stop listening for USB)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: kotlin

   mUSBMonitor?.unregister()

Release USB Monitor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: kotlin

   mUSBMonitor?.destroy()