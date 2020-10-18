
#include "ros/ros.h"
#include "ros/console.h"

#include "dynamo_msgs/SteeringStepper.h"

#define STEER_MAGNITUDE 0.5
#define STEER_PERIOD 1

#define LOOP_RATE 100

double lastSteer = STEER_MAGNITUDE;
ros::Time lastSteerTime;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_steering");
  ros::NodeHandle nh;

  ros::Publisher steerPub = nh.advertise<dynamo_msgs::SteeringStepper>("cmd_steering_angle", 100);

  lastSteerTime = ros::Time::now();

  ros::Rate rate(LOOP_RATE);

  while(ros::ok()) {
    ros::spinOnce();

    if((ros::Time::now()-lastSteerTime).toSec() > STEER_PERIOD){
      dynamo_msgs::SteeringStepper steeringMsg;
      steeringMsg.steering_stepper_engaged = 1;
      steeringMsg.steering_angle = -lastSteer;
      steerPub.publish(steeringMsg);

      lastSteerTime = ros::Time::now();
      lastSteer = -lastSteer;
    }

    rate.sleep();
  }
}
