cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
project(lidar_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  velodyne_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  message_generation
)

add_message_files(FILES
  cloud.msg
  cloudsAndPlane.msg
  point.msg
  obst.msg
  obsts.msg
  gate.msg
  gates.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  message_runtime roscpp std_msgs velodyne_msgs
)

# Setup library
include_directories(
  include ${catkin_INCLUDE_DIRS}
)
set(LIDAR_LIB_SOURCES
    include/${PROJECT_NAME}/classes/Gate.cpp
    include/${PROJECT_NAME}/classes/TrackedGate.cpp
    include/${PROJECT_NAME}/classes/GateTracker.cpp
    include/${PROJECT_NAME}/classes/Hull.cpp
    include/${PROJECT_NAME}/classes/ObstacleHull.cpp
    include/${PROJECT_NAME}/classes/Extreme.cpp
    include/${PROJECT_NAME}/classes/Plane.cpp
    include/${PROJECT_NAME}/classes/Point.cpp
    include/${PROJECT_NAME}/classes/Odometry.cpp
    include/${PROJECT_NAME}/point_plotter.cpp
    include/${PROJECT_NAME}/vlp16_importer.cpp
    include/${PROJECT_NAME}/point_handler.cpp
    include/${PROJECT_NAME}/obstacle_tracker.cpp
    )
add_library(lidar_library ${LIDAR_LIB_SOURCES})
target_link_libraries(lidar_library ${catkin_LIBRARIES})

add_dependencies(lidar_library lidar_package_generate_messages_cpp)

# Node: vlp16_testing
add_executable(vlp16_testing
  src/vlp16_testing.cpp
)
add_dependencies(vlp16_testing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vlp16_testing
  ${catkin_LIBRARIES} lidar_library
)

# Node: vlp16_obst_detection
add_executable(vlp16_obst_detection
  src/vlp16_obst_detection.cpp
)
add_dependencies(vlp16_obst_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vlp16_obst_detection
  ${catkin_LIBRARIES} lidar_library
)

# Node: vlp16_sim_obst_detection
add_executable(vlp16_sim_obst_detection
  src/vlp16_sim_obst_detection.cpp
)
add_dependencies(vlp16_sim_obst_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vlp16_sim_obst_detection
  ${catkin_LIBRARIES} lidar_library
)

# Node: vlp16_graph_test
add_executable(vlp16_graph_test
  src/vlp16_graph_test.cpp
)
add_dependencies(vlp16_graph_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vlp16_graph_test
  ${catkin_LIBRARIES} lidar_library
)

# Node: vlp16_hull_test
add_executable(vlp16_hull_test
  src/vlp16_hull_test.cpp
)
add_dependencies(vlp16_hull_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vlp16_hull_test
  ${catkin_LIBRARIES} lidar_library
)

# Node: gate_classifier
add_executable(gate_classifier
  src/gate_classifier.cpp
)
add_dependencies(gate_classifier ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gate_classifier
  ${catkin_LIBRARIES} lidar_library
)

# Enable external library dependencies
install(TARGETS lidar_library
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


