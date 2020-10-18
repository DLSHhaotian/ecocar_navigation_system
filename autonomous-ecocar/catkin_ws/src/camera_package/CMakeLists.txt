cmake_minimum_required(VERSION 2.8.3)
project(camera_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES still_camera
#  CATKIN_DEPENDS cv_bridge roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_capturing 
  src/image_capturing.cpp
)
target_link_libraries(image_capturing
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(settings_test
  src/settings_test.cpp
)
add_dependencies(settings_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(settings_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)