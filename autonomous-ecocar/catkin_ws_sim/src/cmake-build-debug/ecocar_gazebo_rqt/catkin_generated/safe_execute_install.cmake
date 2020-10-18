execute_process(COMMAND "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/ecocar_gazebo_rqt/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/ecocar_gazebo_rqt/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
