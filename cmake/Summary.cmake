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

include(${CMAKE_CURRENT_LIST_DIR}/IncludeGuard.cmake)
cmake_include_guard()

include(CMakeParseArguments)

# status(TEXT [IF cond text [ELIF cond text] [ELSE cond text]])
macro(status TEXT)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs IF ELIF ELSE)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})

  #message(STATUS "TEXT: ${TEXT}")
  #message(STATUS "THIS_IF: ${THIS_IF}")
  #message(STATUS "THIS_ELIF: ${THIS_ELIF}")
  #message(STATUS "THIS_ELSE: ${THIS_ELSE}")

  set(__msg_list "${TEXT}")
  set(__continue TRUE)

  if(__continue AND DEFINED THIS_IF)
    #message(STATUS "-- THIS_IF: ${THIS_IF}")
    list(LENGTH THIS_IF __if_len)
    if(${__if_len} GREATER 1)
      list(GET THIS_IF 0 __if_cond)
      if(${__if_cond})
        list(REMOVE_AT THIS_IF 0)
        list(APPEND __msg_list ${THIS_IF})
        set(__continue FALSE)
      endif()
    else()
      message(FATAL_ERROR "status() IF must have cond and text, >= 2 items")
    endif()
  endif()

  if(__continue AND DEFINED THIS_ELIF)
    #message(STATUS "-- THIS_ELIF: ${THIS_ELIF}")
    list(LENGTH THIS_ELIF __elif_len)
    if(${__elif_len} GREATER 1)
      list(GET THIS_ELIF 0 __elif_cond)
      if(${__elif_cond})
        list(REMOVE_AT THIS_ELIF 0)
        list(APPEND __msg_list ${THIS_ELIF})
        set(__continue FALSE)
      endif()
    else()
      message(FATAL_ERROR "status() ELIF must have cond and text, >= 2 items")
    endif()
  endif()

  if(__continue AND DEFINED THIS_ELSE)
    #message(STATUS "-- THIS_ELSE: ${THIS_ELSE}")
    list(LENGTH THIS_ELSE __else_len)
    if(${__else_len} GREATER 0)
      list(APPEND __msg_list ${THIS_ELSE})
    else()
      message(FATAL_ERROR "status() ELSE must have text, >= 1 items")
    endif()
  endif()

  string(REPLACE ";" "" __msg_list "${__msg_list}")
  message(STATUS "${__msg_list}")
endmacro()

status("")
status("Platform:")
status("  HOST_OS: ${HOST_OS}")
status("  HOST_NAME: ${HOST_NAME}")
status("  HOST_ARCH: ${HOST_ARCH}")
status("  HOST_COMPILER: ${CMAKE_CXX_COMPILER_ID}")
status("    COMPILER_VERSION: ${CMAKE_CXX_COMPILER_VERSION}")

status("")
status("OpenCV: " IF WITH_OPENCV "YES" ELSE "NO")
if(WITH_OPENCV AND OpenCV_VERSION)
  status("  OpenCV_VERSION: ${OpenCV_VERSION}")
  status("  OpenCV_WORLD: " IF WITH_OPENCV_WORLD "YES" ELSE "NO")
endif()

status("")
status("JPEG: " IF WITH_JPEG "YES" ELSE "NO")
if(WITH_JPEG AND JPEG_VERSION)
  status("  JPEG_VERSION: ${JPEG_VERSION}")
endif()

status("")
