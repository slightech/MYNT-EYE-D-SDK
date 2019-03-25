.. _cmake:

How to use SDK with CMake
=========================

This tutorial will create a project with CMake to start using SDK.

   You could find the project demo in ``<sdk>/platforms/projects/cmake``
   directory.

Preparation
-----------

-  Windows: install the win pack of SDK
-  Linux: build from source and ``make install``

Create Project
--------------

Add ``CMakeLists.txt`` and ``mynteyed_demo.cc`` files,

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.0)

   project(mynteyed_demo VERSION 1.0.0 LANGUAGES C CXX)

   # flags

   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

   ## mynteyed_demo

   add_executable(mynteyed_demo mynteyed_demo.cc)

Config Project
--------------

Add ``mynteyed`` and ``OpenCV`` packages to ``CMakeLists.txt``,

.. code-block:: cmake

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

Add ``include_directories`` and ``target_link_libraries`` to
``mynteyed_demo`` target,

.. code-block:: cmake

   # targets

   include_directories(
     ${OpenCV_INCLUDE_DIRS}
   )

   ## mynteyed_demo

   add_executable(mynteyed_demo mynteyed_demo.cc)
   target_link_libraries(mynteyed_demo mynteye_depth ${OpenCV_LIBS})

Start using SDK
---------------

Include the headers of SDK and start using its APIs, could see the
project demo.

Windows
~~~~~~~

See :ref:`build_win` to “Install Build Tools”.

Then open “x64 Native Tools Command Prompt for VS 2017” command shell to
build and run,

.. code-block:: bash

   mkdir _build
   cd _build

   cmake -G "Visual Studio 15 2017 Win64" ..

   msbuild.exe ALL_BUILD.vcxproj /property:Configuration=Release

   .\Release\mynteyed_demo.exe

Linux
~~~~~

Open “Terminal” to build and run,

.. code-block:: bash

   mkdir _build
   cd _build/

   cmake ..

   make

   ./mynteyed_demo
