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

set(CUR_DIR ${CMAKE_CURRENT_LIST_DIR})

# OS specific instructions in CMAKE: How to?
#   https://stackoverflow.com/questions/9160335/os-specific-instructions-in-cmake-how-to
if(MSVC OR MSYS OR MINGW)
  set(OS_WIN TRUE)
  set(HOST_OS Win)
  add_definitions(-DOS_WIN)
elseif(APPLE)
  set(OS_MAC TRUE)
  set(HOST_OS Mac)
elseif(UNIX)
  set(OS_LINUX TRUE)
  set(HOST_OS Linux)
else()
  message(FATAL_ERROR "Unsupported OS.")
endif()

set(HOST_NAME "${HOST_OS}")

if(OS_LINUX)
  execute_process(COMMAND uname -a COMMAND tr -d '\n' OUTPUT_VARIABLE UNAME_A)
  string(TOLOWER "${UNAME_A}" UNAME_A)
  if(${UNAME_A} MATCHES ".*(tegra|jetsonbot).*")
    set(OS_TEGRA TRUE)
    set(HOST_NAME Tegra)
  elseif(${UNAME_A} MATCHES ".*ubuntu.*")
    set(OS_UBUNTU TRUE)
    set(HOST_NAME Ubuntu)
  endif()
endif()

include(${CMAKE_CURRENT_LIST_DIR}/TargetArch.cmake)
target_architecture(HOST_ARCH)
message(STATUS "HOST_ARCH: ${HOST_ARCH}")

if(CMAKE_COMPILER_IS_GNUCC)
  # GCC 5 changes, Dual ABI: https://gcc.gnu.org/gcc-5/changes.html#libstdcxx
  #add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
endif()

# set_outdir(ARCHIVE outdir
#            LIBRARY outdir
#            RUNTIME outdir
#            KEEP_CONFIG)
macro(set_outdir)
  set(options KEEP_CONFIG)
  set(oneValueArgs ARCHIVE LIBRARY RUNTIME)
  set(multiValueArgs)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(THIS_ARCHIVE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${THIS_ARCHIVE})
  endif()
  if(THIS_LIBRARY)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${THIS_LIBRARY})
  endif()
  if(THIS_RUNTIME)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${THIS_RUNTIME})
  endif()

  if(NOT THIS_KEEP_CONFIG)
    foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})
      string(TOUPPER ${CONFIG} CONFIG)
      set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
      set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
      set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    endforeach()
  endif()
endmacro()

# target_link_threads

# cmake and libpthread
#   https://stackoverflow.com/questions/1620918/cmake-and-libpthread
find_package(Threads QUIET)

# target_link_threads(NAME)
macro(target_link_threads NAME)
  if(THREADS_HAVE_PTHREAD_ARG)
    target_compile_options(PUBLIC ${NAME} "-pthread")
  endif()
  if(CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(${NAME} "${CMAKE_THREAD_LIBS_INIT}")
  endif()
endmacro()

# target_create_scripts

set(__exe2bat_relative_path false)

macro(exe2bat exe_name exe_dir dll_search_paths)
  message(STATUS "Generating ${exe_name}.bat")
  set(__exe_name ${exe_name})
  if(__exe2bat_relative_path)
    set(__dll_relative_search_paths "")
    foreach(path ${dll_search_paths})
      file(RELATIVE_PATH __relative_path "${exe_dir}" "${path}")
      file(TO_NATIVE_PATH ${__relative_path} __relative_path)
      list(APPEND __dll_relative_search_paths ${__relative_path})
    endforeach()
    set(__dll_search_paths "${__dll_relative_search_paths}")
  else()
    set(__dll_native_search_paths "")
    foreach(path ${dll_search_paths})
      file(TO_NATIVE_PATH ${path} __native_path)
      list(APPEND __dll_native_search_paths ${__native_path})
    endforeach()
    set(__dll_search_paths "${__dll_native_search_paths}")
  endif()
  configure_file(
    "${CUR_DIR}/templates/exe.bat.in"
    "${exe_dir}/${__exe_name}.bat"
  )
endmacro()

# target_create_scripts(NAME
#                       [BIN_DIR bin_dir]
#                       [DLL_SEARCH_PATHS path1 path2 ...])
macro(target_create_scripts NAME)
  set(options)
  set(oneValueArgs BIN_DIR)
  set(multiValueArgs DLL_SEARCH_PATHS)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})
  if(NOT THIS_BIN_DIR)
    set(THIS_BIN_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  endif()

  if(OS_WIN)
    exe2bat("${NAME}" "${THIS_BIN_DIR}" "${THIS_DLL_SEARCH_PATHS}")
  endif()
endmacro()

# make_shared_library(NAME
#                     [SRCS src1 src2 ...]
#                     [LINK_LIBS lib1 lib2 ...]
#                     [OUT_DIR outdir])
macro(make_shared_library NAME)
  message(STATUS "Generating shared library ${NAME}")

  set(options)
  set(oneValueArgs OUT_DIR)
  set(multiValueArgs SRCS LINK_LIBS)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_library(${NAME} SHARED ${THIS_SRCS})

  if(THIS_LINK_LIBS)
    target_link_libraries(${NAME} ${THIS_LINK_LIBS})
  endif()

  if(MINGW)
    # MSVC and MinGW DLLs
    #   http://www.mingw.org/wiki/msvc_and_mingw_dlls
    # How to use libraries compiled with MingW in MSVC?
    #   https://stackoverflow.com/questions/2529770/how-to-use-libraries-compiled-with-mingw-in-msvc
    if(NOT THIS_OUT_DIR)
      set(THIS_OUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    endif()
    file(MAKE_DIRECTORY ${THIS_OUT_DIR})
    set_target_properties(${NAME} PROPERTIES
      LINK_FLAGS "-Wl,--output-def,${THIS_OUT_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}${NAME}.def"
    )
  endif()
endmacro()

# make_executable(NAME
#                 [SRCS src1 src2 ...]
#                 [LINK_LIBS lib1 lib2 ...]
#                 [WITH_THREAD]
#                 [DLL_SEARCH_PATHS path1 path2 ...]
#                 [OUT_DIR outdir])
macro(make_executable NAME)
  message(STATUS "Generating executable ${NAME}")

  set(options WITH_THREAD)
  set(oneValueArgs OUT_DIR)
  set(multiValueArgs SRCS LINK_LIBS DLL_SEARCH_PATHS)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_executable(${NAME} ${THIS_SRCS})

  if(THIS_LINK_LIBS)
    target_link_libraries(${NAME} ${THIS_LINK_LIBS})
  endif()

  if(THIS_WITH_THREAD)
    target_link_threads(${NAME})
  endif()

  target_create_scripts(${NAME} DLL_SEARCH_PATHS ${THIS_DLL_SEARCH_PATHS})

  if(THIS_OUT_DIR)
    set_target_properties(${NAME} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY          "${THIS_OUT_DIR}"
      RUNTIME_OUTPUT_DIRECTORY_DEBUG    "${THIS_OUT_DIR}"
      RUNTIME_OUTPUT_DIRECTORY_RELEASE  "${THIS_OUT_DIR}"
    )
  endif()
endmacro()
