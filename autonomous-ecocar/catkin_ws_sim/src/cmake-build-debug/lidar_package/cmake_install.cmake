# Install script for directory: /home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_package/msg" TYPE FILE FILES
    "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloud.msg"
    "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/cloudsAndPlane.msg"
    "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/point.msg"
    "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obst.msg"
    "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/obsts.msg"
    "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gate.msg"
    "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/msg/gates.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_package/cmake" TYPE FILE FILES "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/lidar_package/catkin_generated/installspace/lidar_package-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/devel/include/lidar_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/devel/share/roseus/ros/lidar_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/devel/share/common-lisp/ros/lidar_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/devel/share/gennodejs/ros/lidar_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/devel/lib/python2.7/dist-packages/lidar_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/devel/lib/python2.7/dist-packages/lidar_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/lidar_package/catkin_generated/installspace/lidar_package.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_package/cmake" TYPE FILE FILES "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/lidar_package/catkin_generated/installspace/lidar_package-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_package/cmake" TYPE FILE FILES
    "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/lidar_package/catkin_generated/installspace/lidar_packageConfig.cmake"
    "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/lidar_package/catkin_generated/installspace/lidar_packageConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_package" TYPE FILE FILES "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/lidar_package/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblidar_library.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblidar_library.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblidar_library.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/dlsh/gazebo/autonomous-ecocar/catkin_ws_sim/src/cmake-build-debug/devel/lib/liblidar_library.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblidar_library.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblidar_library.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblidar_library.so"
         OLD_RPATH "/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblidar_library.so")
    endif()
  endif()
endif()

