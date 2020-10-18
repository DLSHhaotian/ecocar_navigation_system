cmake_minimum_required(VERSION 2.8.3)
project(auto_navi)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	dynamo_msgs 
	genmsg 
	message_generation
	tf2
	tf2_ros
)

find_package(Eigen3 REQUIRED)

## Declare ROS messages and services
add_message_files(FILES motorMsg.msg)
add_message_files(FILES driveMsg.msg)
add_message_files(FILES commandMsg.msg)
add_message_files(FILES flagMsg.msg)
add_message_files(FILES emergencyMsg.msg)
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS message_runtime
	DEPENDS Boost EIGEN3
)

## Build talker and listener
include_directories(include 
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

add_executable(voronoi_node src/voronoi_node.cpp)
target_link_libraries(voronoi_node ${catkin_LIBRARIES})
add_dependencies(voronoi_node auto_navi_generate_messages_cpp dynamo_msgs_generate_messages_cpp)

add_executable(parking_node src/parking_node.cpp)
target_link_libraries(parking_node ${catkin_LIBRARIES})
add_dependencies(parking_node auto_navi_generate_messages_cpp dynamo_msgs_generate_messages_cpp)

add_executable(linedrive src/linedrive.cpp)
target_link_libraries(linedrive ${catkin_LIBRARIES})
add_dependencies(linedrive auto_navi_generate_messages_cpp dynamo_msgs_generate_messages_cpp)

add_executable(gates_run src/gates_run.cpp)
target_link_libraries(gates_run ${catkin_LIBRARIES})
add_dependencies(gates_run auto_navi_generate_messages_cpp dynamo_msgs_generate_messages_cpp)

add_executable(braketester src/braketester.cpp)
target_link_libraries(braketester ${catkin_LIBRARIES})
add_dependencies(braketester auto_navi_generate_messages_cpp dynamo_msgs_generate_messages_cpp)

add_executable(motor_node src/motor_node.cpp)
target_link_libraries(motor_node ${catkin_LIBRARIES})
add_dependencies(motor_node auto_navi_generate_messages_cpp dynamo_msgs_generate_messages_cpp)

add_executable(drivepoint_test src/drivepoint_test.cpp)
target_link_libraries(drivepoint_test ${catkin_LIBRARIES})
add_dependencies(drivepoint_test dynamo_msgs_generate_messages_cpp)

add_executable(follow_barrier src/follow_barrier.cpp)
target_link_libraries(follow_barrier ${catkin_LIBRARIES})
add_dependencies(follow_barrier lidar_library dynamo_msgs_generate_messages_cpp)

add_executable(mission_control src/mission_control.cpp)
target_link_libraries(mission_control ${catkin_LIBRARIES})
add_dependencies(mission_control auto_navi_generate_messages_cpp dynamo_msgs_generate_messages_cpp)
