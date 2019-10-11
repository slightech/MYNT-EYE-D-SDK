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

project(mynteyed VERSION 1.8.0 LANGUAGES C CXX)

include(cmake/Common.cmake)

if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX d)
endif()

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

# config

set(MYNTEYE_NAME "${PROJECT_NAME}")
set(MYNTEYE_ROOT "${PROJECT_SOURCE_DIR}")

if(NOT MYNTEYE_NAMESPACE)
  set(MYNTEYE_NAMESPACE "${MYNTEYE_NAME}")
endif()

configure_file(
  include/mynteyed/stubs/global_config.h.in
  include/mynteyed/stubs/global_config.h @ONLY
)

# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

include(cmake/DetectCXX11.cmake)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

string(STRIP "${CMAKE_C_FLAGS}" CMAKE_C_FLAGS)
string(STRIP "${CMAKE_CXX_FLAGS}" CMAKE_CXX_FLAGS)
message(STATUS "C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# packages

LIST(APPEND CMAKE_MODULE_PATH ${MYNTEYE_ROOT}/cmake)

set(OpenCV_FIND_QUIET TRUE)
include(${MYNTEYE_ROOT}/cmake/DetectOpenCV.cmake)

set(JPEG_FIND_QUIET TRUE)
include(${MYNTEYE_ROOT}/cmake/DetectJPEG.cmake)

find_package(eSPDI REQUIRED)

# rpath

set(CMAKE_MACOSX_RPATH 1)
set(MYNTEYE_CMAKE_RPATH "${CMAKE_INSTALL_PREFIX}/lib/3rdparty")
if(WITH_OPENCV)
  message(STATUS "OpenCV_LIB_PATH: ${OpenCV_LIB_PATH}")
  list(APPEND MYNTEYE_CMAKE_RPATH ${OpenCV_LIB_PATH})
endif()
set(CMAKE_INSTALL_RPATH "${MYNTEYE_CMAKE_RPATH}")

# targets

set(OUTPUT_DIR "${MYNTEYE_ROOT}/_output")

set_outdir(
  ARCHIVE ${OUTPUT_DIR}/lib
  LIBRARY ${OUTPUT_DIR}/lib
  RUNTIME ${OUTPUT_DIR}/bin
)

if(WITH_OPENCV)
  include_directories(
    ${OpenCV_INCLUDE_DIRS}
  )
endif()
if(WITH_JPEG)
  include_directories(
    ${JPEG_INCLUDE_DIR}
  )
endif()
include_directories(
  ${eSPDI_INCLUDE_DIRS}
  ${MYNTEYE_ROOT}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

add_definitions(-DMYNTEYE_EXPORTS)

## mynteye_depth

set(MYNTEYE_DEPTH mynteye_depth)

set(MYNTEYE_DEPTH_SRCS
  src/mynteyed/data/channels.cc
  src/mynteyed/device/convertor.cc
  src/mynteyed/device/data_caches.cc
  src/mynteyed/device/device_info.cc
  src/mynteyed/device/device.cc
  src/mynteyed/device/image.cc
  src/mynteyed/device/open_params.cc
  src/mynteyed/device/stream_info.cc
  src/mynteyed/device/types.cc
  src/mynteyed/stubs/types_calib.cc
  src/mynteyed/util/rate.cc
  src/mynteyed/util/strings.cc
  src/mynteyed/camera.cc
  src/mynteyed/types_data.cc
  src/mynteyed/utils.cc
  src/mynteyed/internal/camera_p.cc
  src/mynteyed/internal/image_utils.cc
  src/mynteyed/internal/streams.cc
  src/mynteyed/internal/match.cc
  src/mynteyed/internal/motions.cc
  src/mynteyed/internal/location.cc
  src/mynteyed/internal/distance.cc
  src/mynteyed/filter/base_filter.cpp
  src/mynteyed/filter/filter_spigot.cpp
  src/mynteyed/filter/spatial_filter.cpp
  src/mynteyed/filter/temporal_filter.cpp
)
if(OS_WIN)
  list(APPEND MYNTEYE_DEPTH_SRCS
    src/mynteyed/data/hid/hid_win.cc
    src/mynteyed/device/win/device_win.cc
    src/mynteyed/device/win/color_palette_generator.cc
  )
else()
  list(APPEND MYNTEYE_DEPTH_SRCS
    src/mynteyed/data/hid/hid_linux.cc
    src/mynteyed/device/linux/color_palette_generator.cc
    src/mynteyed/device/linux/device_linux.cc
  )
endif()

if(OS_WIN)
  set(MYNTEYE_LINK_LIBS ${eSPDI_LIBS})
else()
  set(MYNTEYE_LINK_LIBS ${eSPDI_LIBS} "usb" -lpthread)
endif()

if(WITH_OPENCV)
  list(APPEND MYNTEYE_LINK_LIBS ${OpenCV_LIBS})
endif()
if(WITH_JPEG)
  list(APPEND MYNTEYE_LINK_LIBS ${JPEG_LIBRARIES})
endif()

make_shared_library(${MYNTEYE_DEPTH}
  SRCS ${MYNTEYE_DEPTH_SRCS}
  LINK_LIBS ${MYNTEYE_LINK_LIBS}
)

target_include_directories(${MYNTEYE_DEPTH} PUBLIC
  "$<BUILD_INTERFACE:${MYNTEYE_ROOT}/src>"
  "$<INSTALL_INTERFACE:include>"
)

set_target_properties(${MYNTEYE_DEPTH} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

# copy 3rdparty libraries

if(OS_WIN)
  set(__3rdparty_out_dir ${OUTPUT_DIR}/bin/3rdparty)
else()
  set(__3rdparty_out_dir ${OUTPUT_DIR}/lib/3rdparty)
endif()

set(__expr ".so*")
if(OS_WIN)
  set(__expr "*.dll")
elseif(OS_MAC)
  set(__expr "*.dylib")
endif()

if(eSPDI_LIBRARY)
  set(__espdi_libs "")
  set(__espdi_symlinks "")
  list(APPEND eSPDI_LIBS ${eSPDI_3PARTY_LIBS})
  foreach(__espdi_lib ${eSPDI_LIBS})
    string(REGEX MATCH "^${MYNTEYE_ROOT}/*" __match "${__espdi_lib}")
    if(__match)
      string(REGEX REPLACE "\\.[^.]*$" "${__expr}" __espdi_lib ${__espdi_lib})
      file(GLOB __espdi_lib2 "${__espdi_lib}")

      foreach(__espdi_lib ${__espdi_lib2})
        if(IS_SYMLINK "${__espdi_lib}")
          list(APPEND __espdi_symlinks ${__espdi_lib})
        else()
          list(APPEND __espdi_libs ${__espdi_lib})
        endif()
      endforeach()
    endif()
  endforeach()

  # Copy libs before symlinks
  list(APPEND __espdi_libs ${__espdi_symlinks})

  message(STATUS "Copy eSPDI_LIBS to ${__3rdparty_out_dir}")
  foreach(__espdi_lib ${__espdi_libs})
    get_filename_component(__espdi_name ${__espdi_lib} NAME)
    if(EXISTS "${__3rdparty_out_dir}/${__espdi_name}")
      message(STATUS "  Skip ${__espdi_lib}")
    else()
      message(STATUS "  Copy ${__espdi_lib}")
      file(COPY ${__espdi_lib} DESTINATION ${__3rdparty_out_dir})
    endif()
  endforeach()
endif()

# ocvinfo.sh

configure_file(
  "${MYNTEYE_ROOT}/cmake/templates/ocvinfo.sh.in"
  "${MYNTEYE_ROOT}/ocvinfo.sh"
)

# pkginfo.sh

configure_file(
  "${MYNTEYE_ROOT}/cmake/templates/pkginfo.sh.in"
  "${MYNTEYE_ROOT}/pkginfo.sh"
)

# platforms/linux/sdk.cfg

set(SDK_OpenCV_LIB_DIR ${OpenCV_LIB_PATH})

configure_file(
  "${MYNTEYE_ROOT}/cmake/templates/platforms/linux/sdk.cfg.in"
  "${MYNTEYE_ROOT}/platforms/linux/sdk.cfg"
)

# winpack.nsi

if(OS_WIN)
  configure_file(
    scripts/win/nsis/winpack.nsi.in
    ${CMAKE_CURRENT_SOURCE_DIR}/winpack.nsi @ONLY
  )
endif()

# install

set(MYNTEYE_CMAKE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/${MYNTEYE_NAME}")
set(MYNTEYE_CMAKE_BINDIR "${CMAKE_INSTALL_PREFIX}/bin")
set(MYNTEYE_CMAKE_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")
set(MYNTEYE_CMAKE_INSTALLDIR "${MYNTEYE_CMAKE_LIBDIR}/cmake/${MYNTEYE_NAME}")

install(DIRECTORY
    "${MYNTEYE_ROOT}/include"
    "${CMAKE_CURRENT_BINARY_DIR}/include"
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

#message(STATUS "__3rdparty_out_dir: ${__3rdparty_out_dir}")
if(OS_WIN)
  set(__3rdparty_dst_dir ${MYNTEYE_CMAKE_BINDIR})
else()
  set(__3rdparty_dst_dir ${MYNTEYE_CMAKE_LIBDIR})
endif()
install(DIRECTORY "${__3rdparty_out_dir}"
  DESTINATION "${__3rdparty_dst_dir}"
  FILES_MATCHING PATTERN "*.so*" PATTERN "*.dll" PATTERN "*.dylib"
)

install(TARGETS ${MYNTEYE_DEPTH}
  EXPORT mynteyed-targets
  ARCHIVE DESTINATION ${MYNTEYE_CMAKE_LIBDIR}
  LIBRARY DESTINATION ${MYNTEYE_CMAKE_LIBDIR}
  RUNTIME DESTINATION ${MYNTEYE_CMAKE_BINDIR}
)

## cmake files

include(CMakePackageConfigHelpers)

configure_package_config_file(mynteyed-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/mynteyed-config.cmake
  INSTALL_DESTINATION ${MYNTEYE_CMAKE_INSTALLDIR}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(mynteyed-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/mynteyed-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/mynteyed-config-version.cmake
  DESTINATION ${MYNTEYE_CMAKE_INSTALLDIR}
)

install(EXPORT mynteyed-targets
  #NAMESPACE mynteyed::
  DESTINATION ${MYNTEYE_CMAKE_INSTALLDIR}
)

# summary

include(${MYNTEYE_ROOT}/cmake/Summary.cmake)
