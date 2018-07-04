cmake_minimum_required(VERSION 2.8)
project(mynteye_samples)

get_filename_component(PRO_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

include(${PRO_DIR}/cmake/Common.cmake)

# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

include(${PRO_DIR}/cmake/DetectCXX11.cmake)

# variables

set(SDK_DIR ${PRO_DIR})
if(EXISTS ${PRO_DIR}/output)
  set(SDK_DIR ${PRO_DIR}/output)
endif()

set(SDK_INCLUDE_DIRS ${SDK_DIR}/include)
if(NOT EXISTS ${SDK_DIR}/include)
  set(SDK_INCLUDE_DIRS ${PRO_DIR}/modules/core/include/api)
endif()

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

set(SAMPLES_DIR ${PROJECT_SOURCE_DIR})
set(SAMPLES_OUTPUT_DIR ${SAMPLES_DIR}/build/output)

# output

set_outdir(
  ${SAMPLES_OUTPUT_DIR}/lib
  ${SAMPLES_OUTPUT_DIR}/lib
  ${SAMPLES_OUTPUT_DIR}/bin
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

# samples
include_directories(
  ${SDK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

make_executable(get_image
  SRCS src/get_image.cc
  LINKLIBS
    ${OpenCV_LIBS} ${SDK_LIB_CORE}
)

make_executable(get_depth
  SRCS src/get_depth.cc
  LINKLIBS
    ${OpenCV_LIBS} ${SDK_LIB_CORE}
)

find_package(PCL QUIET)
if(PCL_FOUND)

include_directories(${PCL_INCLUDE_DIRS})

make_executable(get_points
  SRCS src/get_points.cc
  LINKLIBS
    ${OpenCV_LIBS} ${SDK_LIB_CORE} ${PCL_LIBRARIES}
)
endif()