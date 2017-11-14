cmake_minimum_required(VERSION 2.8)
project(mynteye)

set(MYNTEYE_VERSION_MAJOR 1)
set(MYNTEYE_VERSION_MINOR 0)

get_filename_component(PRO_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

include(${PRO_DIR}/cmake/Common.cmake)

# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

include(${PRO_DIR}/cmake/DetectCXX11.cmake)

# variables

set(OUTPUT_DIR "${PRO_DIR}/output")

set(MODULES_DIR ${PROJECT_SOURCE_DIR})
set(MODULES_OUTPUT_DIR "${MODULES_DIR}/build/output")

# output

set_outdir(
  ${OUTPUT_DIR}/lib
  ${OUTPUT_DIR}/lib
  ${OUTPUT_DIR}/bin
)

# options

option(DEBUG "Enable Debug Log" OFF)
option(TIMECOST "Enable Time Cost" OFF)

add_definitions(-DMYNTEYE_EXPORTS)
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

find_package(eSPDI REQUIRED)

# modules

include_directories(
  ${MODULES_DIR}
  ${MODULES_DIR}/common
)

add_subdirectory(${MODULES_DIR}/core)

# config

configure_file(
  "${PROJECT_SOURCE_DIR}/mynteye_config.h.in"
  "${PROJECT_SOURCE_DIR}/mynteye_config.h"
)

# copy 3rdparty libraries

if(OS_WIN)
  set(__lib_dir ${PRO_DIR}/output/bin/3rdparty)
else()
  set(__lib_dir ${PRO_DIR}/output/lib/3rdparty)
endif()

set(__expr ".so*")
if(OS_WIN)
  set(__expr "*.dll")
elseif(OS_MAC)
  set(__expr "*.dylib")
endif()

if(eSPDI_LIBRARY)
  set(__espdi_path ${eSPDI_LIBRARY})
  if(OS_WIN)
    string(REGEX REPLACE "\\.[^.]*$" ".dll" __espdi_path ${__espdi_path})
  endif()

  message(STATUS "Copy eSPDI_LIBRARY to ${__lib_dir}")

  get_filename_component(__espdi_name ${__espdi_path} NAME)
  if(EXISTS "${__lib_dir}/${__espdi_name}")
    message(STATUS "  Skip ${__espdi_path}")
  else()
    message(STATUS "  Copy ${__espdi_path}")
    file(COPY ${__espdi_path} DESTINATION ${__lib_dir})
  endif()

  #configure_file(${__espdi_path} ${__lib_dir}/${__espdi_name} COPYONLY)
endif()
