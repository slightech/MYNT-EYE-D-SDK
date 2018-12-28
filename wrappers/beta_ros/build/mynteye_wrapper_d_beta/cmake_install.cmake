# Install script for directory: /home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mynteye_wrapper_d_beta/msg" TYPE FILE FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mynteye_wrapper_d_beta/cmake" TYPE FILE FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/build/mynteye_wrapper_d_beta/catkin_generated/installspace/mynteye_wrapper_d_beta-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/devel/include/mynteye_wrapper_d_beta")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/devel/share/roseus/ros/mynteye_wrapper_d_beta")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/devel/share/common-lisp/ros/mynteye_wrapper_d_beta")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/devel/share/gennodejs/ros/mynteye_wrapper_d_beta")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/devel/lib/python2.7/dist-packages/mynteye_wrapper_d_beta")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/devel/lib/python2.7/dist-packages/mynteye_wrapper_d_beta")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/build/mynteye_wrapper_d_beta/catkin_generated/installspace/mynteye_wrapper_d_beta.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mynteye_wrapper_d_beta/cmake" TYPE FILE FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/build/mynteye_wrapper_d_beta/catkin_generated/installspace/mynteye_wrapper_d_beta-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mynteye_wrapper_d_beta/cmake" TYPE FILE FILES
    "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/build/mynteye_wrapper_d_beta/catkin_generated/installspace/mynteye_wrapper_d_betaConfig.cmake"
    "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/build/mynteye_wrapper_d_beta/catkin_generated/installspace/mynteye_wrapper_d_betaConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mynteye_wrapper_d_beta" TYPE FILE FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmynteye_wrapper_d_beta.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmynteye_wrapper_d_beta.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmynteye_wrapper_d_beta.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/devel/lib/libmynteye_wrapper_d_beta.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmynteye_wrapper_d_beta.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmynteye_wrapper_d_beta.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmynteye_wrapper_d_beta.so"
         OLD_RPATH "/opt/ros/kinetic/lib:/usr/local/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/home/mynt/mynt/mynt-eye-d-sdk/3rdparty/eSPDI/linux/x64:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmynteye_wrapper_d_beta.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mynteye_wrapper_d_beta/mynteye_wrapper_d_beta_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mynteye_wrapper_d_beta/mynteye_wrapper_d_beta_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mynteye_wrapper_d_beta/mynteye_wrapper_d_beta_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mynteye_wrapper_d_beta" TYPE EXECUTABLE FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/devel/lib/mynteye_wrapper_d_beta/mynteye_wrapper_d_beta_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mynteye_wrapper_d_beta/mynteye_wrapper_d_beta_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mynteye_wrapper_d_beta/mynteye_wrapper_d_beta_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mynteye_wrapper_d_beta/mynteye_wrapper_d_beta_node"
         OLD_RPATH "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/devel/lib:/opt/ros/kinetic/lib:/usr/local/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/home/mynt/mynt/mynt-eye-d-sdk/3rdparty/eSPDI/linux/x64:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mynteye_wrapper_d_beta/mynteye_wrapper_d_beta_node")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mynteye_wrapper_d_beta" TYPE DIRECTORY FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/launch")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mynteye_wrapper_d_beta" TYPE DIRECTORY FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/rviz")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mynteye_wrapper_d_beta" TYPE FILE FILES "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/nodelet_plugins.xml")
endif()

