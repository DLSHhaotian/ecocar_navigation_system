
#include "ros/ros.h"

#include "dynamo_msgs/BrakeStepper.h"
#include "dynamo_msgs/SteeringStepper.h"
#include "dynamo_msgs/TeensyWrite.h"
#include "dynamo_msgs/TeensyRead.h"

#include "ecocar_gazebo_msgs/MotorData.h"

#include "sensor_msgs/LaserScan.h"

#define LOOP_RATE 100

#define MINSPEED 2
#define MAXSPEED 4

int burn = 0;
double omega = 0;

double vel = 0;
double lastwheeldist = 0;

double leftdist = 0;
double rightdist = 0;

ros::Time lastTime;

/*
void motorCallback(const ecocar_gazebo_msgs::MotorDataPtr& msg){
  omega = msg->omega_wheel;
  //ROS_INFO("recieved motor msg");
}
*/

// rewrite this speed calculation
// it's borken
// it can't detect standstill
// try and calculate the average instead
void teensyCallback(const dynamo_msgs::TeensyReadPtr& msg){
  ros::Time now = ros::Time::now();

  double dt = (now-lastTime).toSec();
  leftdist = msg->distance_left;
  rightdist = msg->distance_right;
  //ROS_INFO("dt: %f",dt);
  if(dt>0.2){
    vel = (msg->distance_wheel - lastwheeldist)/dt;
    lastwheeldist = msg->distance_wheel;
    lastTime = now;
    ROS_INFO("V: %f", vel);
  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "demo_node");
  ROS_INFO("Starting demo node");

  ros::NodeHandle nh;

  //ros::Subscriber motorSub = nh.subscribe("sim/debug/motor", 1, motorCallback);
  ros::Subscriber teensySub = nh.subscribe("teensy_read", 1, teensyCallback);

  ros::Publisher teensyPub = nh.advertise<dynamo_msgs::TeensyWrite>("teensy_write", 100);
  ros::Publisher brakePub = nh.advertise<dynamo_msgs::BrakeStepper>("cmd_brake_power", 100);
  ros::Publisher steerPub = nh.advertise<dynamo_msgs::SteeringStepper>("cmd_steering_angle", 100);

  lastTime = ros::Time::now();

  ros::Rate rate(LOOP_RATE);

  while(ros::ok()) {
    ros::spinOnce();

    double d = leftdist - rightdist;
    //ROS_INFO("diff: %f", d);

    // Steering angle
    double angle = 1.0*d;
    dynamo_msgs::SteeringStepper steeringMsg;
    steeringMsg.steering_stepper_engaged = 1;
    steeringMsg.steering_angle = angle;
    steerPub.publish(steeringMsg);


    // Motor
    //double radius = 0.2794;
    //double v = radius*omega;

    if(vel > MAXSPEED && burn){
      burn = 0;
      ROS_INFO("Burn: %i", burn);
    } else if(vel < MINSPEED && !burn) {
      burn = 1;
      ROS_INFO("Burn: %i", burn);
    }
    dynamo_msgs::TeensyWrite teensyMsg;
    teensyMsg.burn = burn;
    teensyPub.publish(teensyMsg);

    rate.sleep();
  }

}
