#!/bin/bash
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
gnome-terminal \
-e 'bash -c "rosrun piksi piksi_node; tput bold; echo Program closed.; read line"'
##GPS node
gnome-terminal \
-e 'bash -c "rosrun navigation estimate_pose; tput bold; echo Program closed.; read line"'
## Lightware node
gnome-terminal \
-e 'bash -c "rosrun lightware_sf40_ros sf40_node; tput bold; echo Program closed.; read line"'
## -----------------------------------------------------------------------------

## Record rosbag file subscribing to all published sensor topics
gnome-terminal \
-e 'bash -c "rosbag record --output-name=$now /teensy_read /gyro_angle /gps/fix /gps/rtkfix /gps/time /gps/imu_raw /laser_scan /laser_alarms /car_pose_estimate; tput bold; echo Program closed.; read line"'

