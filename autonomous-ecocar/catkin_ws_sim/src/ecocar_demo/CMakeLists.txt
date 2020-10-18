cmake_minimum_required(VERSION 2.8.3)
project(ecocar_demo)

find_package(catkin REQUIRED COMPONENTS
  message_generation roscpp dynamo_msgs std_msgs ecocar_gazebo_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime ecocar_gazebo_msgs std_msgs dynamo_msgs sensor_msgs
)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_executable(ecocar_demo
  src/ecocar_demo.cpp
)

add_dependencies(ecocar_demo dynamo_msgs_generate_messages_cpp)

target_link_libraries(ecocar_demo ${catkin_LIBRARIES})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
