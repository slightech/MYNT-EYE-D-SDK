cmake_minimum_required(VERSION 2.8.3)
project(mynteye_wrapper)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

function(checkPackage package customMessage)
  set(varName "${package}_FOUND")
  if (NOT "${${varName}}")
    string(REPLACE "_" "-" aptPackage ${package})
    if("${customMessage}" STREQUAL "")
      message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
    else()
      message(FATAL_ERROR "\n\n ${customMessage} \n\n")
    endif()
  endif()
endfunction(checkPackage)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  roscpp
  sensor_msgs
  std_msgs
)

checkPackage("cv_bridge" "")
checkPackage("image_transport" "")
checkPackage("nodelet" "")
checkPackage("roscpp" "")
checkPackage("sensor_msgs" "")
checkPackage("std_msgs" "")

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  if(OpenCV_VERSION VERSION_LESS 3.0)
    add_definitions(-DUSE_OPENCV2)
  else()
    add_definitions(-DUSE_OPENCV3)
  endif()
else()
  message(FATAL_ERROR "OpenCV not found. Allow set OpenCV_DIR to find it.")
endif()

get_filename_component(SDK_DIR "${PROJECT_SOURCE_DIR}/../../../.." ABSOLUTE)
if(EXISTS ${SDK_DIR}/output)
  set(SDK_DIR ${SDK_DIR}/output)
endif()
message(STATUS "SDK_DIR: ${SDK_DIR}")

set(SDK_INCLUDE_DIRS ${SDK_DIR}/include)
set(SDK_LIB_DIR ${SDK_DIR}/lib)
set(SDK_CORE_LIBRARY ${SDK_LIB_DIR}/libmynteye_core.so)
set(SDK_LIBS ${SDK_CORE_LIBRARY})

catkin_package()

# targets

add_compile_options(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SDK_INCLUDE_DIRS}
)

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${SDK_LIBS}
)

add_executable(mynteye_listener src/mynteye_listener.cc)
target_link_libraries(mynteye_listener ${LINK_LIBRARIES})

add_library(mynteye_wrapper src/mynteye_wrapper_nodelet.cc)
target_link_libraries(mynteye_wrapper ${LINK_LIBRARIES})

add_executable(mynteye_wrapper_node src/mynteye_wrapper_node.cc)
target_link_libraries(mynteye_wrapper_node mynteye_wrapper ${LINK_LIBRARIES})

# install

install(TARGETS mynteye_wrapper mynteye_wrapper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
