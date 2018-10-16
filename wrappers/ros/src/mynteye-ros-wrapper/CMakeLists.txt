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
  message_runtime
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf
)

checkPackage("cv_bridge" "")
checkPackage("image_transport" "")
checkPackage("nodelet" "")
checkPackage("roscpp" "")
checkPackage("sensor_msgs" "")
checkPackage("std_msgs" "")

add_message_files(
  FILES
  Temp.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

get_filename_component(SDK_DIR "${PROJECT_SOURCE_DIR}/../../../.." ABSOLUTE)

LIST(APPEND CMAKE_PREFIX_PATH ${SDK_DIR}/_install/lib/cmake)
find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")
if(NOT mynteye_WITH_OPENCV)
  message(FATAL_ERROR "Please install OpenCV, then rebuild mynteye with it."
      " Because these samples use OpenCV to show images.")
endif()

include(${SDK_DIR}/cmake/DetectOpenCV.cmake)

# targets

add_compile_options(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  mynteye_core
)

add_executable(mynteye_listener src/mynteye_listener.cc)
target_link_libraries(mynteye_listener ${LINK_LIBRARIES})

add_library(mynteye_wrapper
  src/mynteye_wrapper_nodelet.cc
  src/pointcloud_generator.cc
)
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
