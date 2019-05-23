.. _install_exe_win:

Windows EXE Installation
========================

   Download here: mynteye-d-x.x.x-win-x64-opencv-3.4.3.exe `Google
   Drive <https://drive.google.com/open?id=1FQrRdpK51U43ihX5pVkMRUedtOOc0FNg>`__,
   `Baidu Pan <https://pan.baidu.com/s/1GeeZ-4-DVyZJ2wUh0aknjQ>`__

After you install the win pack of SDK, there will be a shortcut to the
SDK root directory on your desktop.

First, you should plug the MYNT® EYE camera in a USB 3.0 port.

Second, goto the "\bin\samples" directory and
click “get_image.exe” to run.

Finally, you will see the window that display the realtime frame of the
camera.

.. note::

  If you cannot run samples successfully, please check if the system variable PATH was successfully added ``<SDK_ROOT_DIR>\bin`` , ``<SDK_ROOT_DIR>\bin\3rdparty`` ,
  ``<SDK_ROOT_DIR>\3rdparty\libjpeg-turbo64\bin`` .

Generate samples project of Visual Studio 2017
----------------------------------------------

First, you should install Visual Studio 2017
https://visualstudio.microsoft.com/ and CMake https://cmake.org/.

Second, goto the “\samples” directory and click
“generate.bat” to run.

Finally, you could click ``_build\mynteye_samples.sln`` to open the
samples project.

p.s. The example result of “generate.bat”,

.. code-block:: bat

   -- The C compiler identification is MSVC 19.15.26732.1
   -- The CXX compiler identification is MSVC 19.15.26732.1
   -- Check for working C compiler: C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/VC/Tools/MSVC/14.15.26726/bin/Hostx86/x64/cl.exe
   -- Check for working C compiler: C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/VC/Tools/MSVC/14.15.26726/bin/Hostx86/x64/cl.exe -- works
   -- Detecting C compiler ABI info
   -- Detecting C compiler ABI info - done
   -- Check for working CXX compiler: C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/VC/Tools/MSVC/14.15.26726/bin/Hostx86/x64/cl.exe
   -- Check for working CXX compiler: C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/VC/Tools/MSVC/14.15.26726/bin/Hostx86/x64/cl.exe -- works
   -- Detecting CXX compiler ABI info
   -- Detecting CXX compiler ABI info - done
   -- Detecting CXX compile features
   -- Detecting CXX compile features - done
   -- HOST_ARCH: x86_64
   -- Visual Studio >= 2010, MSVC >= 10.0
   -- C_FLAGS: /DWIN32 /D_WINDOWS /W3 -Wall -O3
   -- CXX_FLAGS: /DWIN32 /D_WINDOWS /W3 /GR /EHsc -Wall -O3
   -- Found mynteye: 1.3.6
   -- OpenCV ARCH: x64
   -- OpenCV RUNTIME: vc15
   -- OpenCV STATIC: OFF
   -- Found OpenCV: C:/Users/John/AppData/Roaming/Slightech/MYNTEYED/SDK/1.3.6/3rdparty/opencv/build (found version "3.4.3")
   -- Found OpenCV 3.4.3 in C:/Users/John/AppData/Roaming/Slightech/MYNTEYED/SDK/1.3.6/3rdparty/opencv/build/x64/vc15/lib
   -- You might need to add C:\Users\John\AppData\Roaming\Slightech\MYNTEYED\SDK\1.3.6\3rdparty\opencv\build\x64\vc15\bin to your PATH to be able to run your applications.
   -- Generating executable get_image
   -- Generating get_image.bat
   -- Generating executable get_depth
   -- Generating get_depth.bat
   -- Generating executable get_imu
   -- Generating get_imu.bat
   -- Configuring done
   -- Generating done
   CMake Warning:
     Manually-specified variables were not used by the project:

       CMAKE_BUILD_TYPE


   -- Build files have been written to: C:/Users/John/AppData/Roaming/Slightech/MYNTEYED/SDK/1.3.6/samples/_build
   Press any key to continue . . .

.. tip::

  Right click sample and select ``Set as StartUp Project``，then launch with Release x64 mode.

