# CMake 如何使用 SDK {#cmake}

本教程将使用 CMake 创建一个项目来使用 SDK 。

> 你可以在 `<sdk>/platforms/projects/cmake` 目录下找到工程样例。

## 准备

* Windows: 安装 SDK 的 exe 包
* Linux: 使用源代码编译和 `make install`

## 创建项目

添加 `CMakeLists.txt` 和 `mynteyed_demo.cc` 文件，

```cmake
cmake_minimum_required(VERSION 3.0)

project(mynteyed_demo VERSION 1.0.0 LANGUAGES C CXX)

# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

## mynteyed_demo

add_executable(mynteyed_demo mynteyed_demo.cc)
```

## 配置项目

增加 `mynteyed` 和 `OpenCV` 到 `CMakeLists.txt` ，

```cmake
# packages

if(MSVC)
  set(SDK_ROOT "$ENV{MYNTEYED_SDK_ROOT}")
  if(SDK_ROOT)
    message(STATUS "MYNTEYED_SDK_ROOT: ${SDK_ROOT}")
    list(APPEND CMAKE_PREFIX_PATH
      "${SDK_ROOT}/lib/cmake"
      "${SDK_ROOT}/3rdparty/opencv/build"
    )
  else()
    message(FATAL_ERROR "MYNTEYED_SDK_ROOT not found, please install SDK firstly")
  endif()
endif()

## mynteyed

find_package(mynteyed REQUIRED)
message(STATUS "Found mynteye: ${mynteyed_VERSION}")

# When SDK build with OpenCV, we can add WITH_OPENCV macro to enable some
# features depending on OpenCV, such as ToMat().
if(mynteyed_WITH_OPENCV)
  add_definitions(-DWITH_OPENCV)
endif()

## OpenCV

# Set where to find OpenCV
#set(OpenCV_DIR "/usr/share/OpenCV")

# When SDK build with OpenCV, we must find the same version here.
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")
```

将 `include_directories` 和 `target_link_libraries` 添加到 `mynteyed_demo` 目标，

```cmake
# targets

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

## mynteyed_demo

add_executable(mynteyed_demo mynteyed_demo.cc)
target_link_libraries(mynteyed_demo mynteye_depth ${OpenCV_LIBS})
```

## 使用SDK

可以参考工程样例添加头文件和使用 API 。

### Windows

可以参考 \link build_win Quick Start Guide for Windows \endlink 安装编译工具。

然后打开 "x64 Native Tools Command Prompt for VS 2017" 命令行来编译和运行，

```bash
mkdir _build
cd _build

cmake -G "Visual Studio 15 2017 Win64" ..

msbuild.exe ALL_BUILD.vcxproj /property:Configuration=Release

.\Release\mynteyed_demo.exe
```

### Linux

打开命令行来编译和运行，

```bash
mkdir _build
cd _build/

cmake ..

make

./mynteyed_demo
```