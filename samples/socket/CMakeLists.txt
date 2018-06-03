cmake_minimum_required(VERSION 2.8)
project(sockets)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(sample_server
  sample/sample_server.cpp
  socket_server.cpp
)

add_executable(sample_client
  sample/sample_client.cpp
  socket_client.cpp
)

target_link_libraries(sample_server ${SDK_LIB_CORE} ${OpenCV_LIBS})
target_link_libraries(sample_client ${SDK_LIB_CORE} ${OpenCV_LIBS})
