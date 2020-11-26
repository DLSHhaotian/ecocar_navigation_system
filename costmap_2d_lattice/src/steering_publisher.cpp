//
// Created by dlsh on 2020/11/24.
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "dynamo_msgs/SteeringStepper.h"

ros::Publisher steerPub;
dynamo_msgs::SteeringStepper steeringmsg;
void latticesteeringCallback(const dynamo_msgs::SteeringStepper& msg);
void latticesteeringCallback(const dynamo_msgs::SteeringStepper& msg) {
    steeringmsg.steering_angle = msg.steering_angle;
    steeringmsg.steering_stepper_engaged = msg.steering_stepper_engaged;

}
int main(int argc, char **argv) {
    ros::init(argc, argv, "steering_publisher");
    ros::NodeHandle nh;

    ros::Subscriber sub_steering_goal = nh.subscribe("cmd_steering_angle_goal", 10, latticesteeringCallback);

    steerPub = nh.advertise<dynamo_msgs::SteeringStepper>("cmd_steering_angle", 100);
    float loopRate = 100; // loop rate in [Hz]
    ros::Rate r(loopRate);
    double steering_angle;
    while (ros::ok()) {
        ros::spinOnce();
        steerPub.publish(steeringmsg);
        r.sleep();

    }
    return 0;
}
