
#include "ros/ros.h"
#include "ros/console.h"

#include "dynamo_msgs/TeensyRead.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"

#include "ecocar_gazebo_msgs/WheelData.h"
#include "std_msgs/Float32.h"

#include <ignition/math.hh>

#include "derivative_estimator.h"

#include <cstdlib>
#include <ctime>

#define LOOP_RATE 1000

#define SPEED_CALC_MAX_SPEED 15 // m/s
#define SPEED_CALC_TIME_CONSTANT 0.2 // sec

double wheeldist = 0;
double frontdist = 0;
double leftdist = 0;
double rightdist = 0;

// Speed calc variables
double lastDistance = 0;
double filteredDistance = 0;
DerivativeEstimator velocityEstimator(SPEED_CALC_TIME_CONSTANT);

unsigned int unpublished = 0;

ros::Publisher gyroPub;
ros::Publisher teensyPub;

ros::Time lastSpeedCalcTime;
ros::Time last_left_lidar_time, last_right_lidar_time, last_dist_time;

ros::Time lastSonarSpikeTime;
double nextSonarSpikeDt = 0;

void teensyPublish(){
  // How else can we merge the data?
  //ROS_INFO("Times of messages merged: %f, %f, %f", last_left_lidar_time.toSec(), last_right_lidar_time.toSec(), last_dist_time.toSec());
  dynamo_msgs::TeensyRead teensyMsg;

  teensyMsg.header.stamp = last_dist_time;
  // All fields of TeensyRead message
  teensyMsg.millis = 0; // could put something
  teensyMsg.speed_wheel = velocityEstimator.value()*3.6; // speed, km/h
  teensyMsg.brake_pressure_1 = 0;
  teensyMsg.brake_pressure_2 = 0;
  teensyMsg.brake_encoder = 0;
  teensyMsg.steering_potentiometer = 0;
  teensyMsg.steering_encoder = 0;
  teensyMsg.temperature_oil = 0;
  teensyMsg.temperature_water = 0;
  teensyMsg.lambda = 0;
  teensyMsg.gear = 0; // I could output this
  teensyMsg.distance_wheel = filteredDistance;
  teensyMsg.distance_left = leftdist;
  teensyMsg.distance_right = rightdist;
  teensyMsg.distance_front = frontdist;
  teensyMsg.energy_used = 0;
  teensyMsg.magnetometer = 0;
  teensyMsg.autopilot = 0;
  teensyMsg.batteryVoltage = 0;
  teensyMsg.engineBatteryVoltage = 0;
  teensyMsg.emergencyButton = 0;

  teensyPub.publish(teensyMsg);

}

void frontSonarCallback(const sensor_msgs::LaserScanPtr& msg){
  frontdist = msg->ranges[0];
  //unpublished++;
  //teensyPublish();

  if(frontdist > msg->range_max){
    frontdist = msg->range_max;
  }

  // Simulate random spikes to max value
  if((ros::Time::now()-lastSonarSpikeTime).toSec() > nextSonarSpikeDt){
    frontdist = msg->range_max;

    lastSonarSpikeTime = ros::Time::now();
    nextSonarSpikeDt = 0.5 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(0.5)));
  }

  //ROS_INFO("Front sonar dist: %f", frontdist);
}

void leftLidarCallback(const sensor_msgs::LaserScanPtr& msg){
  //ROS_INFO("Got left lidar msg");
  last_left_lidar_time = msg->header.stamp;
  leftdist = msg->ranges[0];

  if(leftdist > msg->range_max){
    leftdist = msg->range_max;
  }
  //unpublished++;
  //teensyPublish();
  //ROS_INFO("Left dist: %f", leftdist);
}

void rightLidarCallback(const sensor_msgs::LaserScanPtr& msg){
  //ROS_INFO("Got right lidar msg");
  last_right_lidar_time = msg->header.stamp;
  rightdist = msg->ranges[0];

  if(rightdist > msg->range_max){
    rightdist = msg->range_max;
  }
  //unpublished++;
  //teensyPublish();
  //ROS_INFO("Right dist: %f", rightdist);
}

void wheelCallback(const ecocar_gazebo_msgs::WheelDataPtr& msg){
  //ROS_INFO("Got dist msg: %f", msg->distance_wheel);
  last_dist_time = msg->header.stamp;
  wheeldist = msg->distance_wheel;

  ros::Time thisTime = last_dist_time;

  // Speed calculation
  double dt = (thisTime - lastSpeedCalcTime).toSec();

  if(wheeldist != lastDistance || dt > 2 * SPEED_CALC_TIME_CONSTANT) {
    //ROS_INFO("Changed dist or timeout. dt: %f", dt);
    if(fabs((wheeldist - lastDistance) / dt) < SPEED_CALC_MAX_SPEED) {
      // Probably not noise, we calculate the speed
      lastSpeedCalcTime = thisTime;
      filteredDistance += (wheeldist - lastDistance);

      velocityEstimator.update(filteredDistance, dt);
      //ROS_INFO("Velocity: %f", velocityEstimator.value());

    } else {
      ROS_WARN("Abnormal spike in wheel data, wuut?");
      // spike in value, sensor problem (or speed is higher than threshold)
	  }
  }

  lastDistance = wheeldist;

  teensyPublish();
}

void imuCallback(const sensor_msgs::ImuPtr& msg){
  std_msgs::Float32 gyroMsg;

  // Convert from quaternions to euler angles (only use z)
  double siny = +2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y);
  double cosy = +1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);
  double yaw = atan2(siny, cosy);

  // convert to degrees, same format as actual gyro
  gyroMsg.data = -yaw/IGN_PI*180.0;
  gyroPub.publish(gyroMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sim_topic_organizer");
  ROS_INFO("Starting simulation topic organization node");

  ros::NodeHandle nh;

  ros::Subscriber leftLidarSub = nh.subscribe("sim/raw/left_lidar", 1, leftLidarCallback);
  ros::Subscriber rightLidarSub = nh.subscribe("sim/raw/right_lidar", 1, rightLidarCallback);
  ros::Subscriber frontSonarSub = nh.subscribe("sim/raw/front_sonar", 1, frontSonarCallback);
  ros::Subscriber imuSub = nh.subscribe("sim/raw/imu", 1, imuCallback);
  ros::Subscriber wheelSub = nh.subscribe("sim/raw/distance_wheel", 1, wheelCallback);

  teensyPub = nh.advertise<dynamo_msgs::TeensyRead>("teensy_read", 100);
  gyroPub = nh.advertise<std_msgs::Float32>("gyro_angle", 100);

  ros::Rate rate(LOOP_RATE);

  // Init random stuff
  srand (static_cast <unsigned> (time(0)));

  nextSonarSpikeDt = 0.1 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(0.4)));


  ros::spin();

}
