# Windows SDK 用户指南

## 1. 安装 SDK 依赖

注意: 因为设备权限的问题，命令执行完成之后，您必须重新拔插设备.
```
make init
```

### 1.1 安装 OpenCV

#### 1.1.1 用预先建立的库安装OpenCV (Recommend)

*更多信息您可以参考 OpenCV 官方文档* (https://docs.opencv.org/3.4.2/d3/d52/tutorial_windows_install.html)

1) 进入 OpenCV 源码页 http://sourceforge.net/projects/opencvlibrary/files/opencv-win/
2) 下载一个您想要安装的安装包. 例如 3.4.2/opencv-3.4.2-vc14_vc15.exe
3) 使用管理员权限运行安装包.
4) 安装完成之后，设置 OpenCV 环境变量并添加到系统的 path 变量中.

#### 1.1.2 设置环境变量

开启 cmd, 输入以下命令:

*将 D:\OpenCV 替换为您自己的解压缩目录*

```
setx -m OPENCV_DIR D:\OpenCV\Build\x64\vc14\lib     (suggested for Visual Studio 2015 - 64 bit Windows)
setx -m OPENCV_DIR D:\OpenCV\Build\x64\vc15\lib     (suggested for Visual Studio 2017 - 64 bit Windows)
```
将 OpenCV bin 路径添加到系统环境变量的 PATH 变量列表中

```
D:\OpenCV\Build\x64\vc14\bin     (suggested for Visual Studio 2015 - 64 bit Windows)
D:\OpenCV\Build\x64\vc15\bin     (suggested for Visual Studio 2017 - 64 bit Windows)
```
### 1.2 安装 MSYS2

1) 从 http://mirrors.ustc.edu.cn/msys2/distrib/x86_64/ 下载并安装

2) 将 bin 目录的路径添加到系统变量的 PATH 变量列表中

```
C:\msys64\usr\bin
```

3) Install make
3) 安装 make

```
pacman -Syu
pacman -S make
```

### 1.3 安装 libjpeg-turbo

1) 从 https://sourceforge.net/projects/libjpeg-turbo/files/ 下载 libjpeg-turbo 并安装

2) 将 bin 目录的路径添加到系统变量的 PATH 变量列表中

```
C:\libjpeg-turbo64\bin
```

### 1.4 安装点云例程依赖的 PCL 库 (可选)

从 https://github.com/PointCloudLibrary/pcl/releases 下载集成安装程序(PCL + dependencies)

## 2. 编译 SDK

打开 "适用于 VS 2017 的 x64 本机工具命令提示" 命令行界面

```
cd <sdk>
make all
```

## 3. 运行例程
Note:: 默认打开矫正后的图像。(跑vio时需要使用原图，跑深度或者点云使用矫正后的图像)

1) get_image 显示左右目的图像和彩色深度图

```
.\samples\_output\bin\get_image.bat
```

2) get_depth 显示左目的图像，16UC1的深度图和鼠标选中的像素的深度值(mm)

```
.\samples\_output\bin\get_depth.bat
```

3) get_points 显示左目的图像，16UC1的深度图和点云

```
.\samples\_output\bin\get_points.bat
```

## 4. 清理

```
cd <sdk>
make cleanall
```
