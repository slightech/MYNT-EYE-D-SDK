# Find the 3rdparty/eSPDI includes and library
#
# This module defines:
#
#  eSPDI_FOUND        -- Set to true, if eSPDI found.
#  eSPDI_INCLUDE_DIRS -- Include directory for eSPDI headers.
#  eSPDI_LIBRARY      -- eSPDI library.
#  eSPDI_LIBS         -- eSPDI all libraries.

set(CMAKE_FIND_LIBRARY_PREFIXES_ORIGIN ${CMAKE_FIND_LIBRARY_PREFIXES})
set(CMAKE_FIND_LIBRARY_SUFFIXES_ORIGIN ${CMAKE_FIND_LIBRARY_SUFFIXES})

if(NOT PRO_DIR)
  set(PRO_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
endif()

set(eSPDI_ROOT ${PRO_DIR}/3rdparty/eSPDI)

find_path(eSPDI_INCLUDE_DIRS
  NAMES eSPDI.h
  PATHS ${eSPDI_ROOT}/include ${CMAKE_EXTRA_INCLUDES}
  NO_SYSTEM_PATH
)

set(CMAKE_FIND_LIBRARY_PREFIXES "")

if(MSVC)
  set(eSPDI_LIBPATH ${eSPDI_ROOT}/win/x64)
  set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
elseif(MINGW)
  set(eSPDI_LIBPATH ${eSPDI_ROOT}/mingw/x64)
  set(CMAKE_FIND_LIBRARY_SUFFIXES ".dll")
elseif(APPLE)
  set(eSPDI_LIBPATH ${eSPDI_ROOT}/mac/x64)
  set(CMAKE_FIND_LIBRARY_SUFFIXES ".dylib")
elseif(UNIX)
  if(ARCH_AARCH64)
    set(eSPDI_LIBPATH ${eSPDI_ROOT}/linux/aarch64)
  else()
    set(eSPDI_LIBPATH ${eSPDI_ROOT}/linux/x64)
  endif()
  set(CMAKE_FIND_LIBRARY_SUFFIXES ".so.3.0.14")
else()
  message(FATAL_ERROR "This platform not support now.")
endif()

find_library(eSPDI_LIBRARY
  NAMES libeSPDI
  PATHS ${eSPDI_LIBPATH} ${CMAKE_EXTRA_LIBRARIES}
  NO_SYSTEM_PATH
)
set(eSPDI_LIBS ${eSPDI_LIBRARY})

if(eSPDI_INCLUDE_DIRS AND eSPDI_LIBS)
  set(eSPDI_FOUND TRUE)
else()
  if(NOT eSPDI_FIND_QUIETLY)
    if(NOT eSPDI_INCLUDE_DIRS)
      message(STATUS "Unable to find eSPDI header files!")
    endif()
    if(NOT eSPDI_LIBS)
      message(STATUS "Unable to find eSPDI library files!")
    endif()
  endif()
endif()

if(eSPDI_FOUND)
  if(NOT eSPDI_FIND_QUIETLY)
    message(STATUS "Found components for eSPDI")
    message(STATUS "eSPDI_INCLUDE_DIRS: ${eSPDI_INCLUDE_DIRS}")
    message(STATUS "eSPDI_LIBS: ${eSPDI_LIBS}")
  endif()
else()
  if(eSPDI_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find eSPDI!")
  endif()
endif()

mark_as_advanced (
  eSPDI_FOUND
  eSPDI_INCLUDE_DIRS
  eSPDI_LIBRARY
  eSPDI_LIBS
)

set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES_ORIGIN})
set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES_ORIGIN})
