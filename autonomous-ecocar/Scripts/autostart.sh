#!/bin/bash
setxkbmap -option kpdl:dot
paplay ~/Music/KDE_Startup_1.ogg 
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal -e 'bash -c "rosrun dynamo_helper dynamo_helper_node; read line"'
