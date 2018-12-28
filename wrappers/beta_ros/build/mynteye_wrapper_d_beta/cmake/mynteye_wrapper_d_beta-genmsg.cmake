# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mynteye_wrapper_d_beta: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imynteye_wrapper_d_beta:/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mynteye_wrapper_d_beta_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg" NAME_WE)
add_custom_target(_mynteye_wrapper_d_beta_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mynteye_wrapper_d_beta" "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mynteye_wrapper_d_beta
  "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mynteye_wrapper_d_beta
)

### Generating Services

### Generating Module File
_generate_module_cpp(mynteye_wrapper_d_beta
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mynteye_wrapper_d_beta
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mynteye_wrapper_d_beta_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mynteye_wrapper_d_beta_generate_messages mynteye_wrapper_d_beta_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg" NAME_WE)
add_dependencies(mynteye_wrapper_d_beta_generate_messages_cpp _mynteye_wrapper_d_beta_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mynteye_wrapper_d_beta_gencpp)
add_dependencies(mynteye_wrapper_d_beta_gencpp mynteye_wrapper_d_beta_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mynteye_wrapper_d_beta_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mynteye_wrapper_d_beta
  "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mynteye_wrapper_d_beta
)

### Generating Services

### Generating Module File
_generate_module_eus(mynteye_wrapper_d_beta
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mynteye_wrapper_d_beta
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mynteye_wrapper_d_beta_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mynteye_wrapper_d_beta_generate_messages mynteye_wrapper_d_beta_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg" NAME_WE)
add_dependencies(mynteye_wrapper_d_beta_generate_messages_eus _mynteye_wrapper_d_beta_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mynteye_wrapper_d_beta_geneus)
add_dependencies(mynteye_wrapper_d_beta_geneus mynteye_wrapper_d_beta_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mynteye_wrapper_d_beta_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mynteye_wrapper_d_beta
  "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mynteye_wrapper_d_beta
)

### Generating Services

### Generating Module File
_generate_module_lisp(mynteye_wrapper_d_beta
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mynteye_wrapper_d_beta
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mynteye_wrapper_d_beta_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mynteye_wrapper_d_beta_generate_messages mynteye_wrapper_d_beta_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg" NAME_WE)
add_dependencies(mynteye_wrapper_d_beta_generate_messages_lisp _mynteye_wrapper_d_beta_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mynteye_wrapper_d_beta_genlisp)
add_dependencies(mynteye_wrapper_d_beta_genlisp mynteye_wrapper_d_beta_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mynteye_wrapper_d_beta_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mynteye_wrapper_d_beta
  "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mynteye_wrapper_d_beta
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mynteye_wrapper_d_beta
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mynteye_wrapper_d_beta
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mynteye_wrapper_d_beta_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mynteye_wrapper_d_beta_generate_messages mynteye_wrapper_d_beta_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg" NAME_WE)
add_dependencies(mynteye_wrapper_d_beta_generate_messages_nodejs _mynteye_wrapper_d_beta_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mynteye_wrapper_d_beta_gennodejs)
add_dependencies(mynteye_wrapper_d_beta_gennodejs mynteye_wrapper_d_beta_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mynteye_wrapper_d_beta_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mynteye_wrapper_d_beta
  "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mynteye_wrapper_d_beta
)

### Generating Services

### Generating Module File
_generate_module_py(mynteye_wrapper_d_beta
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mynteye_wrapper_d_beta
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mynteye_wrapper_d_beta_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mynteye_wrapper_d_beta_generate_messages mynteye_wrapper_d_beta_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mynt/mynt/mynt-eye-d-sdk/wrappers/beta_ros/src/mynteye_wrapper_d_beta/msg/Temp.msg" NAME_WE)
add_dependencies(mynteye_wrapper_d_beta_generate_messages_py _mynteye_wrapper_d_beta_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mynteye_wrapper_d_beta_genpy)
add_dependencies(mynteye_wrapper_d_beta_genpy mynteye_wrapper_d_beta_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mynteye_wrapper_d_beta_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mynteye_wrapper_d_beta)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mynteye_wrapper_d_beta
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mynteye_wrapper_d_beta_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mynteye_wrapper_d_beta)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mynteye_wrapper_d_beta
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mynteye_wrapper_d_beta_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mynteye_wrapper_d_beta)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mynteye_wrapper_d_beta
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mynteye_wrapper_d_beta_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mynteye_wrapper_d_beta)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mynteye_wrapper_d_beta
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mynteye_wrapper_d_beta_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mynteye_wrapper_d_beta)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mynteye_wrapper_d_beta\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mynteye_wrapper_d_beta
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mynteye_wrapper_d_beta_generate_messages_py std_msgs_generate_messages_py)
endif()
