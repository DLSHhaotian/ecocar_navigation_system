#include "ros/ros.h"
//#include "autonomous-ecocar/catkin_ws/src/navigation/global_variables.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "lidar_package/point.h"
#include "lidar_package/obst.h"
#include "lidar_package/obsts.h"
#include <cstdio>
#include <vector>
#include <iostream>
#include <fstream>
#include <ctime>
#include "dynamo_msgs/BrakeStepper.h"
#include "dynamo_msgs/TeensyRead.h"
#include "dynamo_msgs/TeensyWrite.h"
//using namespace std;

// this node is for testing the car's brake speed and brake distance.
int first =1;
ros::Publisher brake_pub;
ros::Publisher teensyPub;
double drivendist, theta, x, y, speed, accel;	// global odo values
double dist_begin=0.0;
double x_start, x_end, x_diff, speed_start, dist_start, dist_end, dist_diff;	// global values for the brake test
int aktivator, counter, brake_completed;
ros::Duration t_diff,t_diff_2,t_diff_3;
ros::Time t1, t2, t_start, t_end, t_stopped, t_burn_complete;
int state = 0;
int burn_complete = 0;
int burn;
double bp1,bp2;
double brake_bar = 5.0;
double dist_burn_complete;

// til motoren
int MINSPEED = 0;
int MAXSPEED = 10;


void carPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_carPoseEstimate) {
	x = msg_carPoseEstimate->data[0];
	y = msg_carPoseEstimate->data[1];
	theta = msg_carPoseEstimate->data[3];
}


void teensyCallback(const dynamo_msgs::TeensyReadPtr& msg) {
	speed = msg->speed_wheel;
	drivendist = msg->distance_wheel;	
	bp1 = msg->brake_pressure_1;
	bp2 = msg->brake_pressure_2;
}



void brakeTestScript() {
	t2 = ros::Time::now();
	t_diff = t2-t1;

	if (brake_completed == 0) {
		std::cout << "\n" << drivendist << " " << speed << " " << burn << " " << MAXSPEED << " " << MINSPEED << " " << aktivator;
		std::cout << " " << brake_bar << " " << bp1 << " " << bp2 << " " << t_diff;
	}

	//std::cout<<"\nstate:" << state << "   drivendist: " << drivendist << " time " << t_diff << "  bc: "<<brake_completed<<"  burn: "<< burn;
	// initialize the test
	if (first && t_diff.toSec() > 1.0) {
		dist_begin = drivendist;
		first=0;
		state = 1;
	}

	// over or under the desired speed limits?
	if (state == 1 && speed > MAXSPEED) {	//over maxspeed
		state = 2;
		burn_complete = 1;	// so that it only burns once
		dist_burn_complete = drivendist;
		t_burn_complete = ros::Time::now();
	}
	if (state == 2 && speed < MINSPEED) state = 1; //under minspeed

	// to burn or not to burn?
	if (state == 1 && burn_complete == 0) burn = 1;
	if (state == 2 || state == 3 ) burn = 0;	
	
	
	// activate the brake
	//if ((drivendist-dist_begin) >= 2.0 && brake_completed == 0 && first == 0 && aktivator == 0 && burn_complete == 1) {
	if ((t2.toSec()-t_burn_complete.toSec()) >= 1.0 && brake_completed == 0 && first == 0 && aktivator == 0 && burn_complete == 1) {
		x_start = x;
		dist_start = drivendist;
		speed_start = speed;
		aktivator = 1;		// this activates the brake further down
		t_start = ros::Time::now();
		//std::cout << "\nbrake activated";
	}

	// publish to brake to activate it
	if (aktivator == 1) {
		dynamo_msgs::BrakeStepper bremse;
		bremse.brake_stepper_engaged = true;
		bremse.brake_power = brake_bar ;	// [bar]
		brake_pub.publish(bremse);
		//std::cout << "\nbrake     drivendist: "<< drivendist << "        speed: " << speed << "       time: " << t_diff;
	}
	/*
	if (speed <= 2.0) {
		counter++;	
	} else {
		counter = 0;
	}
*/
	t_diff_2 = t2-t_start;
	// when the car has stopped
	if (aktivator == 1 && speed < MINSPEED+0.1) {
		aktivator = 0;
		x_end = x;
		x_diff = x_end - x_start;
		dist_end = drivendist;
		dist_diff = dist_end - dist_start;
		t_end = ros::Time::now();
		t_diff = t_end - t_start;		
		//std::cout << "\n\nResults: starting speed: " << speed_start << "     brake distance with " << brake_bar  << " [bar]: " << dist_diff << " [m]    in " << t_diff << "  seconds";
		brake_completed = 1;
		
		// release the brake
		dynamo_msgs::BrakeStepper bremse;
		bremse.brake_stepper_engaged = true;
		bremse.brake_power = (float) 0.0;	// [bar]
		brake_pub.publish(bremse);
		
		//ros::shutdown();	// maybe
	}
	
	// publish to burn or not
	dynamo_msgs::TeensyWrite teensyMsg;
    teensyMsg.burn = burn;
    teensyMsg.autopilot_active = true;
    teensyPub.publish(teensyMsg);

}




int main(int argc, char **argv) {
	// ini
	aktivator = 0;
	brake_completed = 0;
	counter = 0;
	ros::init(argc, argv, "braketester");
	ros::NodeHandle n;
	
	t1 = ros::Time::now();
	
	ros::Subscriber sub = n.subscribe("car_pose_estimate", 1000, carPoseCallback);		// modtag messages på topic "car_pose_estimate" og send til funktionen carPoseCallback
	ros::Subscriber teensySub = n.subscribe("teensy_read", 10, teensyCallback);
	teensyPub = n.advertise<dynamo_msgs::TeensyWrite>("teensy_write", 100);
	brake_pub = n.advertise<dynamo_msgs::BrakeStepper>("cmd_brake_power",10);    // publish msgs of type "brakeStepper" to topic "cmd_brake_power"

	float loopRate = 20; // loop rate in [Hz]
	ros::Rate r(loopRate);

	while (ros::ok()) {		
		ros::spinOnce();
		
		brakeTestScript();
		
		r.sleep();
		
	}

	return 0;
}
