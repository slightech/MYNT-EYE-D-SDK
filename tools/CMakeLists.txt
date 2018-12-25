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

project(mynteye_tools VERSION 1.3.6 LANGUAGES C CXX)

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
message(STATUS "Found mynteye: ${mynteye_VERSION}")
if(NOT mynteyed_WITH_OPENCV)
  message(FATAL_ERROR "Please install OpenCV, then rebuild mynteye with it."
      " Because these samples use OpenCV to show images.")
endif()

include(${PRO_DIR}/cmake/DetectOpenCV.cmake)

# targets

set(OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_output")

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}
)

set(MYNTEYE_DLL_SEARCH_PATHS
  ${PRO_DIR}/_install/bin
  ${PRO_DIR}/_install/bin/3rdparty
  ${OpenCV_LIB_SEARCH_PATH}
)

# dataset

add_subdirectory(dataset)

# writer

add_subdirectory(writer)

# detection

add_subdirectory(detection)
