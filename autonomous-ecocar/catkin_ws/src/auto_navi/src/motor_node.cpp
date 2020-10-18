#include "ros/ros.h"
//#include "autonomous-ecocar/catkin_ws/src/navigation/global_variables.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <vector>
#include <iostream>
#include <fstream>
#include <ctime>
#include "dynamo_msgs/TeensyWrite.h"
#include "dynamo_msgs/TeensyRead.h"
#include "dynamo_msgs/Speak.h"
#include "auto_navi/emergencyMsg.h"
#include "dynamo_msgs/BrakeStepper.h"
#include "auto_navi/motorMsg.h"
#include "geometry_msgs/PolygonStamped.h"

//using namespace std;

// this node is for testing the car's motor and burn function.

double MINSPEED_FAST = 2.0;
double MAXSPEED_FAST = 10.0;

double MINSPEED_SLOW = 3.0;
double MAXSPEED_SLOW = 11.0;

int lastfast = 0;
int fast = 0;

double BW2FW = 1.516;
double x_0, x_1, y_0, y_1, xx, yy, th;
ros::Publisher teensyPub;
ros::Publisher brake_pub;

ros::Publisher speakpub;
double drivendist, theta, speed, accel;
double speed_end;
double odox, odoy, odotheta;

int state = 0;
/* States;
 * 1: Burn
 * 2: Coast
 * 3: Parking
 * 4: Unused, but exists. Coast
*/



int first = 1;
int braking = 0;
int master_aktivator;
ros::Duration t_diff;
ros::Time t1, t2;
double starting_drivendist, drivendiff;
double laserleft, laserright;

double decelbrake = 5.8;
double stopbrake = 10.8;
double brake_bar = 0.0;
int spotdetected = 0;
double pointdist = 20.0;
int spot_detected = 0;
geometry_msgs::Point32 p_front;
geometry_msgs::Point32 p_center;


void carPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_carPoseEstimate) {
	// store values from odometry
	odox = msg_carPoseEstimate->data[0];
	odoy = msg_carPoseEstimate->data[1];
	odotheta = msg_carPoseEstimate->data[3];

}


void pointdister() {
	double odo_front_wheel_x = cos(odotheta) * BW2FW - sin(odotheta) * 0 + odox;
	double odo_front_wheel_y = sin(odotheta) * BW2FW + cos(odotheta) * 0 + odoy;
	pointdist = sqrt(pow(xx - odo_front_wheel_x, 2) + pow(yy - odo_front_wheel_y, 2));
	ROS_INFO("Pointdist: %f", pointdist);
}



void parkingCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg) {
	spot_detected = 1;	// indicate globally that a spot has been detected

	//calculate drive point and theta
	p_center = msg->polygon.points[0];	// center of parking spot
	p_front = msg->polygon.points[1];	// center of front line of parking spot
	y_1 = p_center.y;
	x_1 = p_center.x;
	y_0 = p_front.y;
	x_0 = p_front.x;
	th = atan2(y_1 - y_0, x_1 - x_0);
	xx = x_1;
	yy = y_1;

	ROS_INFO("Received point: x = %f  y = %f  theta = %f", xx, yy, th);
	pointdister();
}




void fastCallback (const auto_navi::motorMsg::ConstPtr& msg) {
	lastfast = fast;
	fast = msg->fast;

	if (fast != lastfast) {
		dynamo_msgs::Speak speakmsg;

		if (fast == 1) {
			speakmsg.textMessage = "Fast";
		} else {
			speakmsg.textMessage = "Slow";
		}
		speakmsg.sound = 0;
		speakpub.publish(speakmsg);
	}
}


void motor_brake_stop(const dynamo_msgs::BrakeStepperPtr& msg) {	// maybe unneccessary
	if (msg->brake_power > 0) braking = 1;
	if (msg->brake_power == 0) braking = 0;
}


void teensyCallback(const dynamo_msgs::TeensyReadPtr& msg) {
	speed = msg->speed_wheel;
	drivendist = msg->distance_wheel;
	laserleft = msg->distance_left;
	laserright = msg->distance_right;
}


void regulateSpeed() {
	t2 = ros::Time::now();

	t_diff = t2 - t1;

	ROS_INFO("Maxspeed_slow: %f    Minspeed_slow: %f    Maxspeed_fast: %f    Minspeed_fast %f", MAXSPEED_SLOW, MINSPEED_SLOW, MAXSPEED_FAST, MINSPEED_FAST);

	// for parking
	if (spot_detected == 1 && pointdist < 3.0) {
		brake_bar = decelbrake;	// brake lightly when approaching the spot coordinate
		state = 3;
	}

// No lasers on left/right side of car (yet?)
//	if ( (laserleft < 2.0 && laserleft > 1.0) || (laserright < 2.0 && laserright > 1.0) ) state == 4;

	if (first && t_diff.toSec() > 2.0) {
		starting_drivendist = drivendist;
		first = 0;
		state = 1;
	}
	drivendiff = drivendist - starting_drivendist;

	// Set speed, depending on the "fast" parameter
	double MINSPEED;
	double MAXSPEED;

	if (fast == 1) {
		MINSPEED = MINSPEED_FAST;
		MAXSPEED = MAXSPEED_FAST;
	}
	if (fast == 0) {
		MINSPEED = MINSPEED_SLOW;
		MAXSPEED = MAXSPEED_SLOW;
	}

	//if (drivendiff > 2000.0) state = 3; //run over
	if (state == 1 && speed > MAXSPEED) state = 2; //over maxspeed, coast
	if (state == 2 && speed < MINSPEED) state = 1; //under minspeed, burn

	// Experimental braking towards point, activates when a parking spot has been found

	if (spot_detected == 1 && pointdist < 1) {
		dynamo_msgs::BrakeStepper bremse;
		bremse.brake_stepper_engaged = true;
		bremse.brake_power = 4.0;
		brake_pub.publish(bremse);
	} else if (spot_detected == 1 && speed > MINSPEED_SLOW + 1.5) {
		dynamo_msgs::BrakeStepper bremse;
		bremse.brake_stepper_engaged = true;
		bremse.brake_power = 1.0;
		brake_pub.publish(bremse);
	} else if (spot_detected == 1) {
		dynamo_msgs::BrakeStepper bremse;
		bremse.brake_stepper_engaged = true;
		bremse.brake_power = 0;
		brake_pub.publish(bremse);
	}

	// Should it burn?
	int burn = 0;
	if (state == 1) burn = 1;
	if (state == 2 || state == 3 || braking || state == 4 ) burn = 0;

	dynamo_msgs::TeensyWrite teensyMsg;
	teensyMsg.burn = burn;
	teensyMsg.autopilot_active = true;
	teensyPub.publish(teensyMsg);

}


void emergencyCallback(const auto_navi::emergencyMsg::ConstPtr& msg) {
	master_aktivator = msg->master_switch;
	ROS_WARN("received shutdown");
	//if (master_aktivator == false) ros::shutdown();
}


int main(int argc, char **argv) {
	// ini
	ros::init(argc, argv, "motor_node");

	ros::NodeHandle n("~");
	n.param<double>("MAXSPEED_SLOW", MAXSPEED_SLOW, MAXSPEED_SLOW);	// so we can pass arguments when calling rosrun
	n.param<double>("MAXSPEED_FAST", MAXSPEED_FAST, MAXSPEED_FAST);
	n.param<double>("MINSPEED_SLOW", MINSPEED_SLOW, MINSPEED_SLOW);
	n.param<double>("MINSPEED_FAST", MINSPEED_FAST, MINSPEED_FAST);

	t1 = ros::Time::now();

	//ros::Subscriber sub = n.subscribe("car_pose_estimate", 1000, carPoseCallback);	// modtag messages på topic "car_pose_estimate" og send til funktionen carPoseCallback
	ros::Subscriber sub2 = n.subscribe("/cmd_master_switch", 10, emergencyCallback);
	ros::Subscriber teensySub = n.subscribe("/teensy_read", 1, teensyCallback);
	ros::Subscriber brakeSub = n.subscribe("/cmd_brake_power", 10, motor_brake_stop);
	ros::Subscriber fastSub = n.subscribe("/fast", 1, fastCallback);
	ros::Subscriber parking_sub = n.subscribe("parking_spot_detection/corners", 10, parkingCallback);
	ros::Subscriber odo_sub = n.subscribe("car_pose_estimate", 1000, carPoseCallback);

	teensyPub = n.advertise<dynamo_msgs::TeensyWrite>("/teensy_write", 100);
	brake_pub = n.advertise<dynamo_msgs::BrakeStepper>("/cmd_brake_power", 10);
	speakpub = n.advertise<dynamo_msgs::Speak>("/speak", 10);

	float loopRate = 100; // loop rate in [Hz]
	ros::Rate r(loopRate);

	while (ros::ok()) {
		ros::spinOnce();

		if (spot_detected == 1) pointdister();
		regulateSpeed();

		r.sleep();

	}
	ROS_INFO("Shutting down motor_node");
	dynamo_msgs::TeensyWrite teensyMsg;
	teensyMsg.burn = false;
	teensyPub.publish(teensyMsg);

	return 0;
}
