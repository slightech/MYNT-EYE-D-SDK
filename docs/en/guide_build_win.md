# SDK Quick Start Guide for Windows

## 1. Install SDK dependencies

### 1.1 Install OpenCV

#### 1.1.1 Install OpenCV with Pre-built Libraries (Recommend)

*For more details you can reference OpenCV offical document* (https://docs.opencv.org/3.4.2/d3/d52/tutorial_windows_install.html)

1) Go to OpenCV Sourceforge page http://sourceforge.net/projects/opencvlibrary/files/opencv-win/
2) Choose a build you want to use and download it. For example 3.4.2/opencv-3.4.2-vc14_vc15.exe
3) Make sure you have admin rights. Unpack the self-extracting archive.
4) To finalize the installation, go to set the OpenCV environment variable and add it to the systems path.

#### 1.1.2 Set up environment variable

Start up a command window and enter:

*Change the D:\OpenCV to your opencv unpack path*

```
setx -m OPENCV_DIR D:\OpenCV\Build\x64\vc14\lib     (suggested for Visual Studio 2015 - 64 bit Windows)
setx -m OPENCV_DIR D:\OpenCV\Build\x64\vc15\lib     (suggested for Visual Studio 2017 - 64 bit Windows)
```
Add OpenCV bin path to System PATH environment variable list

```
D:\OpenCV\Build\x64\vc14\bin     (suggested for Visual Studio 2015 - 64 bit Windows)
D:\OpenCV\Build\x64\vc15\bin     (suggested for Visual Studio 2017 - 64 bit Windows)
```
### 1.2 Install MSYS2

1) Download MSYS2 from http://mirrors.ustc.edu.cn/msys2/distrib/x86_64/ and install

2) Add bin path to System PATH environment variable list
```
C:\msys64\usr\bin
```

3) Install make
```
pacman -Syu
pacman -S make
```

### 1.3 Install libjpeg-turbo

1) Download libjpeg-turbo from https://sourceforge.net/projects/libjpeg-turbo/files/ and install

2) Add bin path to System PATH environment variable list
```
C:\libjpeg-turbo64\bin
```

### 1.4 Install PCL for Point Cloud sample (Optional)

Download All-in-one installers (PCL + dependencies) from:
https://github.com/PointCloudLibrary/pcl/releases

## 2. Build SDK

Open "x64 Native Tools Command Prompt for VS 2017"(适用于 VS 2017 的 x64 本机工具命令提示) command shell

```
cd <sdk>
make all
```

## 3. Run Samples
Note:: Open the rectified image by default (Run vio need to raw image, run depth or points cloud need to rectified image.)

1) get_image shows the left and right camera image and colorful depthmap

```
.\samples\_output\bin\get_image.bat
```

2) get_depth shows the left camera image, 16UC1 depthmap and depth value(mm) on mouse pointed pixal
```
.\samples\_output\bin\get_depth.bat
```

3) get_points shows the left camera image, 16UC1 depthmap and point cloud view
```
.\samples\_output\bin\get_points.bat
```

4) get_imu shows motion datas
```
.\samples\_output\bin\get_imu
```

5) get_img_params show camera intrinsics and save in file
```
.\samples\_output\bin\get_img_params
```

6) get_imu_params show imu intrinsics and save in file
```
.\samples\_output\bin\get_imu_params
```

7) get_from_callbacks show image and imu data by callback
```
.\samples\_output\bin\get_from_callbacks
```

8) get_all_with_options open device with different options
```
.\samples\_output\bin\get_all_with_options
```

## 4. Clean

```
cd <sdk>
make cleanall
```
