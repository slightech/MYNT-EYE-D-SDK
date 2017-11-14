
if(MSVC)

# Support For C++11/14/17 Features (Modern C++)
#   https://msdn.microsoft.com/en-us/library/hh567368.aspx
# MSVC_VERSION:
#   https://cmake.org/cmake/help/latest/variable/MSVC_VERSION.html

if(NOT (MSVC_VERSION LESS 1600))
  add_definitions(-DCXX11)
  message(STATUS "Visual Studio >= 2010, MSVC >= 10.0.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

else()

set(CXX_FLAGS_EXTRA "")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CXX_FLAGS_EXTRA "-std=c++11")
  add_definitions(-DCXX11)
  message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CXX_FLAGS_EXTRA "-std=c++0x")
  add_definitions(-DCXX0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS_EXTRA}")

# Ensure access this in sub directories
set(CXX_FLAGS_EXTRA "${CXX_FLAGS_EXTRA}" CACHE STRING "Value of the extra cxx flags.")

endif()
