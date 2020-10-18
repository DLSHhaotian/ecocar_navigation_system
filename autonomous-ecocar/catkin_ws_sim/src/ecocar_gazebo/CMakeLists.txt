cmake_minimum_required(VERSION 2.8.3)
project(ecocar_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros roscpp dynamo_msgs std_msgs ecocar_gazebo_msgs
)

find_package(gazebo 9 REQUIRED)

#generate_messages(DEPENDENCIES ecocar_gazebo_msgs std_msgs geometry_msgs dynamo_msgs)

catkin_package(
  CATKIN_DEPENDS gazebo_ros message_runtime ecocar_gazebo_msgs std_msgs dynamo_msgs
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${SDFormat_INCLUDE_DIRS})

add_library(EcocarPlugin SHARED plugins/EcocarPlugin.cpp plugins/Motor.cpp plugins/Drag.cpp)
target_link_libraries(EcocarPlugin ${GAZEBO_LIBRARIES})

add_library(TrackPlugin SHARED plugins/TrackPlugin.cpp)
target_link_libraries(TrackPlugin ${GAZEBO_LIBRARIES})

add_library(RoadMarkerPlugin SHARED plugins/RoadMarkerPlugin.cpp)
target_link_libraries(RoadMarkerPlugin ${GAZEBO_LIBRARIES})

add_dependencies(TrackPlugin ecocar_gazebo_msgs_generate_messages_cpp)

add_dependencies(EcocarPlugin dynamo_msgs_generate_messages_cpp ecocar_gazebo_msgs_generate_messages_cpp)

add_executable(topic_organizer
  src/topic_organizer.cpp
)

add_library(derivative_estimator
  include/derivative_estimator.cpp)

add_dependencies(topic_organizer dynamo_msgs_generate_messages_cpp ecocar_gazebo_msgs_generate_messages_cpp derivative_estimator)
target_link_libraries(topic_organizer ${catkin_LIBRARIES} derivative_estimator)

add_executable(test_steering
  src/test_steering.cpp
)

add_dependencies(test_steering dynamo_msgs_generate_messages_cpp)
target_link_libraries(test_steering ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
