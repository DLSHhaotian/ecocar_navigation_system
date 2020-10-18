while true
do

rostopic pub -1 /cmd_steering_angle dynamo_msgs/SteeringStepper "steering_stepper_engaged: true
steering_angle: 20.0"
sleep 2
rostopic pub -1 /cmd_steering_angle dynamo_msgs/SteeringStepper "steering_stepper_engaged: true
steering_angle: -20.0" 
sleep 2

done

