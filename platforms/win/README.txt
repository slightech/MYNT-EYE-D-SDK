# MYNT® EYE Depth SDK

################################################################################
Language: 简体中文
################################################################################

## 如何开始使用 SDK

1) 运行样例程序

安装完 SDK 的 exe 安装包后，桌面会生成 SDK 根目录的快捷方式。

进入 "<SDK_ROOT_DIR>\bin\samples" 目录，双击 "get_image.exe" 运行，即可看到相机画面。

2）生成样例工程

首先，安装好 Visual Studio 2017 <https://visualstudio.microsoft.com/> 和 CMake <https://cmake.org/> 。

接着，进入 "<SDK_ROOT_DIR>\samples" 目录， 双击 "generate.bat" 即可生成样例工程 `_build\mynteye_samples.sln` 。

p.s. "generate.bat" 运行结果，可以参考下方英文内容。

3）工程使用样例

现有 `vs2017`, `qtcreator`, `cmake` 使用 SDK 的工程样例，在 "<SDK_ROOT_DIR>\projects" 目录。

关于如何创建这些工程的教程，请见 SDK 文档。

################################################################################
Language: English
################################################################################

## How to start using SDK

1) Run the prebuilt samples, ensure the SDK works well.

After you install the win pack of SDK, there will be a shortcut to the SDK root directory on your desktop.

First, you should plug the MYNT® EYE camera in a USB 3.0 port.

Second, goto the "<SDK_ROOT_DIR>\bin\samples" directory and click "get_image.exe" to run.

Finally, you will see the window that display the realtime frame of the camera.

2) Generate samples project of Visual Studio 2017.

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

3）How to create a project with SDK.

There are some project demos, including `vs2017` `qtcreator` `cmake`, under "<SDK_ROOT_DIR>\projects" directory.

If you wanna know how to create these projets, please see the SDK documentations.
