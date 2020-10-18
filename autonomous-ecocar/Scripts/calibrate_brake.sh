#!/bin/bash

close_func () {
  echo "Closing.."
  rosnode kill /brake_node
}

trap close_func SIGINT

gnome-terminal -e 'rosrun brake_node brake_lut'
sleep 2
rosservice call /calibrate_brake

echo "Wait for brake actuator to stop moving and press CTRL-C"

cat
