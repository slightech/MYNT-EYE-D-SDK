Open camera
======================

Set the Camera connection callback
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: kotlin

   mCamera?.setCameraListener(object : MYNTCamera.ICameraListener {

       override fun didConnectedCamera(camera: MYNTCamera?) {

       }

       override fun didDisconnectedCamera(camera: MYNTCamera?) {

       }
   })

Connect the camera (the permission dialog box will pop up)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: kotlin

   mCamera?.connect()

After the connection is successful, the data is retrieved
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: kotlin

   //Open equipment
   mCamera?.open()
   // Set IR value
   mCamera?.irCurrentValue = IR_DEFAULT_VALUE

   backgroundHandler?.post {
       if (mCamera == null) return@post
       // Color image previews related objects
       mColorSurface = Surface(colorTextureView.surfaceTexture)
       // Depth image previews related objects
       mDepthSurface = Surface(depthTextureView.surfaceTexture)

       mCamera?.setPreviewDisplay(mDepthSurface, MYNTCamera.Frame.DEPTH)
       mCamera?.setPreviewDisplay(mColorSurface, MYNTCamera.Frame.COLOR)
       // Set preview size (480 / 720)
       mCamera?.setPreviewSize(previewSize.height)
       // Set depth type( 8bit / 11bit)
       mCamera?.setDepthType(depthType)
       // Set the image callback
       mCamera?.setFrameCallback { data ->
           if (data.flag == FrameData.DEPTH) {
               // Depth map 
           }
           if (data.flag == FrameData.COLOR) {
               // Color map
           }
       }
       // To preview
       mCamera?.start(MYNTCamera.Source.ALL, MYNTCamera.Frame.ALL)
   }