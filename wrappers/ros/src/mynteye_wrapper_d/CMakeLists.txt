cmake_minimum_required(VERSION 2.8.3)
project(mynteye_wrapper_d)

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
  message_generation
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf
)

checkPackage("cv_bridge" "")
checkPackage("image_transport" "")
checkPackage("message_generation" "")
checkPackage("nodelet" "")
checkPackage("roscpp" "")
checkPackage("sensor_msgs" "")
checkPackage("std_msgs" "")
checkPackage("tf" "")

add_message_files(FILES Temp.msg)

add_service_files(
  FILES
  GetParams.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport message_runtime nodelet roscpp sensor_msgs std_msgs tf
)

get_filename_component(SDK_DIR "${PROJECT_SOURCE_DIR}/../../../.." ABSOLUTE)

LIST(APPEND CMAKE_PREFIX_PATH ${SDK_DIR}/_install/lib/cmake)
find_package(mynteyed REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")
if(NOT mynteyed_WITH_OPENCV)
  message(FATAL_ERROR "Please install OpenCV, then rebuild mynteye with it."
      " Because these samples use OpenCV to show images.")
endif()

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message(STATUS "OpenCV is found.")
  set(WITH_OPENCV TRUE)
  add_definitions(-DWITH_OPENCV)
  message(STATUS "Defined WITH_OPENCV.")
endif()

# targets

add_compile_options(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  mynteye_depth
)

add_library(mynteye_wrapper_d
  src/mynteye_wrapper_nodelet.cc
  src/pointcloud_generator.cc
)
target_link_libraries(mynteye_wrapper_d ${LINK_LIBRARIES})
add_dependencies(mynteye_wrapper_d ${PROJECT_NAME}_generate_messages_cpp)

add_executable(mynteye_wrapper_d_node src/mynteye_wrapper_node.cc)
target_link_libraries(mynteye_wrapper_d_node mynteye_wrapper_d ${LINK_LIBRARIES})

add_executable(mynteye_listener_d src/mynteye_listener.cc)
target_link_libraries(mynteye_listener_d ${LINK_LIBRARIES})

# install

install(TARGETS mynteye_wrapper_d mynteye_wrapper_d_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
