# Windows EXE Installation {#install_exe_win}

> Download here: mynteye-d-1.6.0-win-x64-opencv-3.4.3.exe [Google Drive](https://drive.google.com/open?id=1IP2kcnpOIWg5wQFhuMIdcToVlWUf0WD2), [百度网盘](https://pan.baidu.com/s/1zKVF4e85zrAq5-cClXnIKQ)

After you install the win pack of SDK, there will be a shortcut to the SDK root directory on your desktop.

First, you should plug the MYNT® EYE camera in a USB 3.0 port.

Second, goto the "<SDK_ROOT_DIR>\bin\samples" directory and click "get_image.exe" to run.

Finally, you will see the window that display the realtime frame of the camera.

## Generate samples project of Visual Studio 2017

First, you should install Visual Studio 2017 <https://visualstudio.microsoft.com/> and CMake <https://cmake.org/>.

Second, goto the "<SDK_ROOT_DIR>\samples" directory and click "generate.bat" to run.

Finally, you could click `_build\mynteye_samples.sln` to open the samples project.

p.s. The example result of "generate.bat",

```cmd
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
```

## Start using MYNT® EYE Depth SDK with Visual Studio 2017

Goto the "<SDK_ROOT_DIR>\projects\vs2017", see the "README.md".
