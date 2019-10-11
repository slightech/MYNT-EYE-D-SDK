# Copyright 2018 Slightech Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.0)

project(mynteye_samples VERSION 1.3.6 LANGUAGES C CXX)

get_filename_component(PRO_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

include(${PRO_DIR}/cmake/Common.cmake)

# options

option(DEBUG "Enable Debug Log" OFF)
option(TIMECOST "Enable Time Cost" OFF)

add_definitions(-DLOG_TAG=MYNTEYE)

if(DEBUG)
  add_definitions(-DDEBUG)
  message(STATUS "Using macro DEBUG")
endif()
if(TIMECOST)
  add_definitions(-DTIME_COST)
  message(STATUS "Using macro TIME_COST")
endif()

# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

include(${PRO_DIR}/cmake/DetectCXX11.cmake)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

string(STRIP "${CMAKE_C_FLAGS}" CMAKE_C_FLAGS)
string(STRIP "${CMAKE_CXX_FLAGS}" CMAKE_CXX_FLAGS)
message(STATUS "C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# packages

LIST(APPEND CMAKE_MODULE_PATH ${PRO_DIR}/cmake)

LIST(APPEND CMAKE_PREFIX_PATH ${PRO_DIR}/_install/lib/cmake)
find_package(mynteyed REQUIRED)
message(STATUS "Found mynteye: ${mynteyed_VERSION}")
if(NOT mynteyed_WITH_OPENCV)
  message(FATAL_ERROR "Please install OpenCV, then rebuild mynteye with it."
      " Because these samples use OpenCV to show images.")
endif()

include(${PRO_DIR}/cmake/DetectOpenCV.cmake)

# samples

set(OUTPUT_DIR "${CMAKE_CURRENT_LIST_DIR}/_output")

set_outdir(
  ARCHIVE ${OUTPUT_DIR}/lib
  LIBRARY ${OUTPUT_DIR}/lib
  RUNTIME ${OUTPUT_DIR}/bin
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/src
)

set(MYNTEYE_DLL_SEARCH_PATHS
  ${PRO_DIR}/_install/bin
  ${PRO_DIR}/_install/bin/3rdparty
  ${OpenCV_LIB_SEARCH_PATH}
)

make_executable(check_frame_rate
  SRCS
    src/check_frame_rate.cc
    src/util/cam_utils.cc
    src/util/cv_painter.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_image
  SRCS
    src/get_image.cc
    src/util/cam_utils.cc
    src/util/cv_painter.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_stereo_image
  SRCS
    src/get_stereo_image.cc
    src/util/cam_utils.cc
    src/util/cv_painter.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_depth
  SRCS
    src/get_depth.cc
    src/util/cam_utils.cc
    src/util/cv_painter.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_depth_with_filter
  SRCS
    src/get_depth_with_filter.cc
    src/util/cam_utils.cc
    src/util/cv_painter.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_imu
  SRCS
    src/get_imu.cc
    src/util/cam_utils.cc
    src/util/cv_painter.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_from_callbacks
  SRCS
    src/get_from_callbacks.cc
    src/util/cam_utils.cc
    src/util/cv_painter.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_all_with_options
  SRCS
    src/get_all_with_options.cc
    src/util/cam_utils.cc
    src/util/cv_painter.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_img_params
  SRCS
    src/get_img_params.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_imu_params
  SRCS
    src/get_imu_params.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_gps
  SRCS
    src/get_gps.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(get_ultrasonic
  SRCS
    src/get_ultrasonic.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

make_executable(auto_restart
  SRCS
    src/auto_restart.cc
    src/util/cam_utils.cc
    src/util/cv_painter.cc
  LINK_LIBS mynteye_depth ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
)

find_package(PCL)
if(PCL_FOUND)
  #list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

  message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
  message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")
  include_directories(${PCL_INCLUDE_DIRS})

  make_executable(get_points
    SRCS
      src/get_points.cc
      src/util/cam_utils.cc
      src/util/cv_painter.cc
      src/util/pc_viewer.cc
    WITH_THREAD
    LINK_LIBS mynteye_depth ${OpenCV_LIBS} ${PCL_LIBRARIES}
    DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
  )

  make_executable(view_points
    SRCS src/points/view_points.cc
    LINK_LIBS ${PCL_LIBRARIES}
  )

  make_executable(get_points2
    SRCS
      src/points/get_points2.cc
      src/util/pc_utils.cc
    LINK_LIBS mynteye_depth ${OpenCV_LIBS} ${PCL_LIBRARIES}
    DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
  )

  make_executable(get_points_with_filter
    SRCS
      src/points/get_points_with_filter.cc
      src/util/pc_utils.cc
    LINK_LIBS mynteye_depth ${OpenCV_LIBS} ${PCL_LIBRARIES}
    DLL_SEARCH_PATHS ${MYNTEYE_DLL_SEARCH_PATHS}
  )
endif()
