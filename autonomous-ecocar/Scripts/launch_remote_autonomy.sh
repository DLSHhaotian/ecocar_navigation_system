#!/bin/bash
gnome-terminal -e 'bash -c "roscore; tput bold; echo Program closed.; read line"'
sleep 2
gnome-terminal \
-e 'bash -c "rosrun remote remote_node; tput bold; echo Program closed.; read line"'
gnome-terminal \
-e 'bash -c "rosrun teensy teensy_node; tput bold; echo Program closed.; read line"'
gnome-terminal \
-e 'bash -c "rosrun phidget phidget_node; tput bold; echo Program closed.; read line"'
gnome-terminal \
-e 'bash -c "rosrun piksi piksi_node; tput bold; echo Program closed.; read line"'
gnome-terminal \
-e 'bash -c "rosrun dynamo_helper dynamo_helper_node; tput bold; echo Program closed.; read line"'
