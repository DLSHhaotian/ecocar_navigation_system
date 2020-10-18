# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "auto_navi: 6 messages, 1 services")

set(MSG_I_FLAGS "-Iauto_navi:/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(auto_navi_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg" NAME_WE)
add_custom_target(_auto_navi_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auto_navi" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg" ""
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg" NAME_WE)
add_custom_target(_auto_navi_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auto_navi" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg" ""
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg" NAME_WE)
add_custom_target(_auto_navi_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auto_navi" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg" ""
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg" NAME_WE)
add_custom_target(_auto_navi_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auto_navi" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg" ""
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg" NAME_WE)
add_custom_target(_auto_navi_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auto_navi" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_auto_navi_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auto_navi" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv" ""
)

get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg" NAME_WE)
add_custom_target(_auto_navi_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "auto_navi" "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auto_navi
)
_generate_msg_cpp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auto_navi
)
_generate_msg_cpp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auto_navi
)
_generate_msg_cpp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auto_navi
)
_generate_msg_cpp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auto_navi
)
_generate_msg_cpp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auto_navi
)

### Generating Services
_generate_srv_cpp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auto_navi
)

### Generating Module File
_generate_module_cpp(auto_navi
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auto_navi
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(auto_navi_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(auto_navi_generate_messages auto_navi_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_cpp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_cpp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_cpp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_cpp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_cpp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(auto_navi_generate_messages_cpp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_cpp _auto_navi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(auto_navi_gencpp)
add_dependencies(auto_navi_gencpp auto_navi_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS auto_navi_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/auto_navi
)
_generate_msg_eus(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/auto_navi
)
_generate_msg_eus(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/auto_navi
)
_generate_msg_eus(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/auto_navi
)
_generate_msg_eus(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/auto_navi
)
_generate_msg_eus(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/auto_navi
)

### Generating Services
_generate_srv_eus(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/auto_navi
)

### Generating Module File
_generate_module_eus(auto_navi
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/auto_navi
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(auto_navi_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(auto_navi_generate_messages auto_navi_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_eus _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_eus _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_eus _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_eus _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_eus _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(auto_navi_generate_messages_eus _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_eus _auto_navi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(auto_navi_geneus)
add_dependencies(auto_navi_geneus auto_navi_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS auto_navi_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auto_navi
)
_generate_msg_lisp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auto_navi
)
_generate_msg_lisp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auto_navi
)
_generate_msg_lisp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auto_navi
)
_generate_msg_lisp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auto_navi
)
_generate_msg_lisp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auto_navi
)

### Generating Services
_generate_srv_lisp(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auto_navi
)

### Generating Module File
_generate_module_lisp(auto_navi
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auto_navi
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(auto_navi_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(auto_navi_generate_messages auto_navi_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_lisp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_lisp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_lisp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_lisp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_lisp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(auto_navi_generate_messages_lisp _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_lisp _auto_navi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(auto_navi_genlisp)
add_dependencies(auto_navi_genlisp auto_navi_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS auto_navi_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/auto_navi
)
_generate_msg_nodejs(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/auto_navi
)
_generate_msg_nodejs(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/auto_navi
)
_generate_msg_nodejs(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/auto_navi
)
_generate_msg_nodejs(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/auto_navi
)
_generate_msg_nodejs(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/auto_navi
)

### Generating Services
_generate_srv_nodejs(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/auto_navi
)

### Generating Module File
_generate_module_nodejs(auto_navi
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/auto_navi
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(auto_navi_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(auto_navi_generate_messages auto_navi_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_nodejs _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_nodejs _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_nodejs _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_nodejs _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_nodejs _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(auto_navi_generate_messages_nodejs _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_nodejs _auto_navi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(auto_navi_gennodejs)
add_dependencies(auto_navi_gennodejs auto_navi_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS auto_navi_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi
)
_generate_msg_py(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi
)
_generate_msg_py(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi
)
_generate_msg_py(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi
)
_generate_msg_py(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi
)
_generate_msg_py(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi
)

### Generating Services
_generate_srv_py(auto_navi
  "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi
)

### Generating Module File
_generate_module_py(auto_navi
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(auto_navi_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(auto_navi_generate_messages auto_navi_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/flagMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_py _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/Num.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_py _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/emergencyMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_py _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/motorMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_py _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/driveMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_py _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(auto_navi_generate_messages_py _auto_navi_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/auto_navi/msg/commandMsg.msg" NAME_WE)
add_dependencies(auto_navi_generate_messages_py _auto_navi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(auto_navi_genpy)
add_dependencies(auto_navi_genpy auto_navi_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS auto_navi_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auto_navi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/auto_navi
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(auto_navi_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/auto_navi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/auto_navi
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(auto_navi_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auto_navi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/auto_navi
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(auto_navi_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/auto_navi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/auto_navi
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(auto_navi_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/auto_navi
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(auto_navi_generate_messages_py std_msgs_generate_messages_py)
endif()
