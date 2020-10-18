cmake_minimum_required(VERSION 2.8.3)
project(steering_node)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_srvs std_msgs dynamo_msgs dynamo_helper)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidget
  CATKIN_DEPENDS roscpp std_msgs std_srvs dynamo_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(phidget_driver
  ##)

## Declare a C++ executable
add_executable(steering_node
  src/steering_node.cpp
)

add_library(FilteredDerivative
  include/FilteredDerivative.cpp)

add_library(CUSUM
  include/CUSUM.cpp)

add_library(SteeringSystem
  include/SteeringSystem.cpp)

add_dependencies(SteeringSystem ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(SteeringSystem
  ${catkin_LIBRARIES}
  LowPass
  FilteredDerivative
  CUSUM
  phidget21
)


add_dependencies(steering_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} dynamo_msgs_generate_messages_cpp)

target_link_libraries(steering_node
  ${catkin_LIBRARIES}
  SteeringSystem
#  phidget21
)
