#!/bin/bash
cd ~/catkin_ws
gnome-terminal -e 'bash -c "roscore; tput bold; echo Program closed.; read line"'
sleep 2
## Run all nodes----------------------------------------------------------------
##Teensy node - contains odometry and side LIDARs
gnome-terminal \
-e 'bash -c "rosrun teensy teensy_node; tput bold; echo Program closed.; read line"'
##Gyro node
gnome-terminal \
-e 'bash -c "rosrun gyro gyro_node; tput bold; echo Program closed.; read line"'
##GPS node
##gnome-terminal \
##-e 'bash -c "rosrun piksi piksi_node; tput bold; echo Program closed.; read line"'
##GPS node
gnome-terminal \
-e 'bash -c "rosrun navigation estimate_pose; tput bold; echo Program closed.; read line"'
## Lightware node
gnome-terminal \
-e 'bash -c "rosrun velodyne_driver velodyne_node _model:=VLP16; tput bold; echo Program closed.; read line"'
## Lightware node
gnome-terminal \
-e 'bash -c "rosrun lidar_package vlp16_obst_detection; tput bold; echo Program closed.; read line"'
## voronoi node
gnome-terminal \
-e 'bash -c "rosrun beginner_tutorials scanmaker; tput bold; echo Program closed.; read line"'
## Line drive node
gnome-terminal \
-e 'bash -c "rosrun beginner_tutorials drive; tput bold; echo Program closed.; read line"'
## -----------------------------------------------------------------------------

gnome-terminal \
-e 'bash -c "rosrun steering_node steering_node; tput bold; echo Program closed.; read line"'
## -----------------------------------------------------------------------------

## Record rosbag file subscribing to all published sensor topics
gnome-terminal \
-e 'bash -c "rosbag record -a; tput bold; echo Program closed.; read line"'

