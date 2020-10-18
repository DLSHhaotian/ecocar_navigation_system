#!/bin/bash

close_func () {
  echo "Closing.."
  rosnode kill /pose_estimation
  rosnode kill /velodyne_node
  rosnode kill /gyro_node
  rosnode kill /usb_cam/image_proc
  rosnode kill /usb_cam
}

trap close_func SIGINT

gnome-terminal -e 'rosrun gyro gyro_node'

gnome-terminal -e 'rosrun navigation estimate_pose'

gnome-terminal -e 'rosrun velodyne_driver velodyne_node _model:=VLP16'

gnome-terminal -e "rosrun usb_cam usb_cam_node _image_width:=2048 _image_height:=1536 _framerate:=15"

gnome-terminal -e 'sh -c "ROS_NAMESPACE=usb_cam rosrun image_proc image_proc"'

gnome-terminal -e 'rosbag record -a -x \"/usb_cam/image_mono/(.*)|/usb_cam/image_rect/(.*)|/usb_cam/image_raw(.*)|/usb_cam/image_color(.*)|/usb_cam/image_proc_debayer(.*)|/usb_cam/image_mono(.*)|/b_cam/image_rect|/usb_cam/image_rect_color|/usb_cam/image_rect_color/theora|/usb_cam/image_proc(.*)|/usb_cam/image_rect|/image_view(.*)\"'

cat
