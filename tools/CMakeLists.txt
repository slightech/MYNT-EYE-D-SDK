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

project(mynteye_tools VERSION 2.0.0 LANGUAGES C CXX)

get_filename_component(PRO_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

include(${PRO_DIR}/cmake/Common.cmake)

# flags
if(OS_WIN)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

include(${PRO_DIR}/cmake/DetectCXX11.cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

string(STRIP "${CMAKE_C_FLAGS}" CMAKE_C_FLAGS)
string(STRIP "${CMAKE_CXX_FLAGS}" CMAKE_CXX_FLAGS)
message(STATUS "C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# variables

set(SDK_DIR ${PRO_DIR})
if(EXISTS ${PRO_DIR}/output)
  set(SDK_DIR ${PRO_DIR}/output)
endif()

set(SDK_INCLUDE_DIRS
  ${SDK_DIR}/include
  ${PRO_DIR}/modules/core/include/api
  ${PRO_DIR}/modules/core/include/public
  ${PRO_DIR}/modules/core/include/hid
)

set(SDK_LIB_DIR ${SDK_DIR}/lib)
if(MINGW)
  set(SDK_LIB_DIR ${SDK_DIR}/bin)
else()
  set(SDK_LIB_DIR ${SDK_DIR}/lib)
endif()
if(OS_WIN AND NOT MINGW)
  set(SDK_LIB_CORE ${SDK_LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}mynteye_core.lib)
else()
  set(SDK_LIB_CORE ${SDK_LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}mynteye_core${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

set(OUTPUT_DIR "${CMAKE_CURRENT_LIST_DIR}/_output")

# output
set_outdir(
  ARCHIVE ${OUTPUT_DIR}/lib
  LIBRARY ${OUTPUT_DIR}/lib
  RUNTIME ${OUTPUT_DIR}/bin
)

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

# required

LIST(APPEND CMAKE_MODULE_PATH ${PRO_DIR}/cmake)

find_package(OpenCV QUIET)
if(OpenCV_FOUND)
  if(OpenCV_VERSION VERSION_LESS 3.0)
    set(USE_OPENCV2 TRUE)
    add_definitions(-DUSE_OPENCV2)
  else()
    set(USE_OPENCV3 TRUE)
    add_definitions(-DUSE_OPENCV3)
  endif()
else()
  message(FATAL_ERROR "OpenCV not found. Allow set OpenCV_DIR to find it.")
endif()

message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

if(OS_WIN)
  get_filename_component(OpenCV_LIB_SEARCH_PATH "${OpenCV_LIB_PATH}/../bin" ABSOLUTE)
else()
  set(OpenCV_LIB_SEARCH_PATH "${OpenCV_LIB_PATH}")
endif()

# tools

include_directories(
  ${SDK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# dataset
add_subdirectory(dataset)




















