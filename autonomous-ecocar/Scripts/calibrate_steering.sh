#!/bin/bash

close_func () {
  echo "Closing.."
  rosnode kill /steering_node
}

trap close_func SIGINT

gnome-terminal -e 'rosrun steering_node steering_node'
sleep 2

rosservice call /calibrate_steering

echo "Calibrating steering limits.."
sleep 20
rosservice call /calibrate_steering_velocity

echo "Wait for steering velocity calibration to stop and press CTRL-C"

cat
