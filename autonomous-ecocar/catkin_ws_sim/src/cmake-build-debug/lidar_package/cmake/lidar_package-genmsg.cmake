# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lidar_package: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ilidar_package:/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lidar_package_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg" NAME_WE)
add_custom_target(_lidar_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_package" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg" "lidar_package/point"
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg" NAME_WE)
add_custom_target(_lidar_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_package" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg" "lidar_package/gate:lidar_package/point"
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg" NAME_WE)
add_custom_target(_lidar_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_package" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg" "lidar_package/point"
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg" NAME_WE)
add_custom_target(_lidar_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_package" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg" "lidar_package/point"
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg" NAME_WE)
add_custom_target(_lidar_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_package" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg" ""
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg" NAME_WE)
add_custom_target(_lidar_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_package" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg" "lidar_package/point:lidar_package/cloud"
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg" NAME_WE)
add_custom_target(_lidar_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_package" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg" "lidar_package/point:lidar_package/obst"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_package
)
_generate_msg_cpp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_package
)
_generate_msg_cpp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_package
)
_generate_msg_cpp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_package
)
_generate_msg_cpp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_package
)
_generate_msg_cpp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_package
)
_generate_msg_cpp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_package
)

### Generating Services

### Generating Module File
_generate_module_cpp(lidar_package
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_package
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lidar_package_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lidar_package_generate_messages lidar_package_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_cpp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_cpp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_cpp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_cpp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_cpp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_cpp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_cpp _lidar_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_package_gencpp)
add_dependencies(lidar_package_gencpp lidar_package_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_package_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_package
)
_generate_msg_eus(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_package
)
_generate_msg_eus(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_package
)
_generate_msg_eus(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_package
)
_generate_msg_eus(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_package
)
_generate_msg_eus(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_package
)
_generate_msg_eus(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_package
)

### Generating Services

### Generating Module File
_generate_module_eus(lidar_package
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_package
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lidar_package_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lidar_package_generate_messages lidar_package_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_eus _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_eus _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_eus _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_eus _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_eus _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_eus _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_eus _lidar_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_package_geneus)
add_dependencies(lidar_package_geneus lidar_package_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_package_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_package
)
_generate_msg_lisp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_package
)
_generate_msg_lisp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_package
)
_generate_msg_lisp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_package
)
_generate_msg_lisp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_package
)
_generate_msg_lisp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_package
)
_generate_msg_lisp(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_package
)

### Generating Services

### Generating Module File
_generate_module_lisp(lidar_package
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_package
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lidar_package_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lidar_package_generate_messages lidar_package_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_lisp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_lisp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_lisp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_lisp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_lisp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_lisp _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_lisp _lidar_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_package_genlisp)
add_dependencies(lidar_package_genlisp lidar_package_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_package_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_package
)
_generate_msg_nodejs(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_package
)
_generate_msg_nodejs(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_package
)
_generate_msg_nodejs(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_package
)
_generate_msg_nodejs(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_package
)
_generate_msg_nodejs(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_package
)
_generate_msg_nodejs(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_package
)

### Generating Services

### Generating Module File
_generate_module_nodejs(lidar_package
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_package
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lidar_package_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lidar_package_generate_messages lidar_package_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_nodejs _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_nodejs _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_nodejs _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_nodejs _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_nodejs _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_nodejs _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_nodejs _lidar_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_package_gennodejs)
add_dependencies(lidar_package_gennodejs lidar_package_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_package_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package
)
_generate_msg_py(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package
)
_generate_msg_py(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package
)
_generate_msg_py(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package
)
_generate_msg_py(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package
)
_generate_msg_py(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package
)
_generate_msg_py(lidar_package
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg"
  "${MSG_I_FLAGS}"
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg;/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package
)

### Generating Services

### Generating Module File
_generate_module_py(lidar_package
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lidar_package_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lidar_package_generate_messages lidar_package_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_py _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_py _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_py _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_py _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_py _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_py _lidar_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg" NAME_WE)
add_dependencies(lidar_package_generate_messages_py _lidar_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_package_genpy)
add_dependencies(lidar_package_genpy lidar_package_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_package_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_package
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(lidar_package_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_package
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(lidar_package_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_package
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(lidar_package_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_package
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(lidar_package_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_package
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(lidar_package_generate_messages_py std_msgs_generate_messages_py)
endif()
