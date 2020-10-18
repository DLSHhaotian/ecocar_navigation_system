#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "auto_navi/driveMsg.h"
#include "dynamo_msgs/SteeringStepper.h"
#include "dynamo_msgs/TeensyWrite.h"
#include "dynamo_msgs/TeensyRead.h"
#include "auto_navi/emergencyMsg.h"

double pose[3] = {0, 0, 0};
double x_y_th[3] = {0, 0, 0};
double vel = 0.0;
double BW2FW = 1.516;
int sim = 0; //simulation variable.
int N = 5; //anglevec size
int master_aktivator, state;
double last_u = 0.0;
double d_u = 22.5 * M_PI / 180.0 * 0.01;
double anglevec [5] = {0.0};
double teensy_left, teensy_right;
double limit = 15.0 * M_PI / 180.0;
bool parking = false;
ros::Publisher steerPub;
ros::Publisher teensyPub;
#define MINSPEED 1
#define MAXSPEED 2

void rosbagCallback (const std_msgs::Float32MultiArray::ConstPtr& msg_pose_est) {
	pose[0] = msg_pose_est->data[0]; //x
	pose[1] = msg_pose_est->data[1]; //y
	pose[2] = msg_pose_est->data[3]; //theta
	vel = msg_pose_est->data[4]; //velocity		// take from teensy insted.
	double globalx = cos(pose[2]) * BW2FW - sin(pose[2]) * (0) + pose[0];
	double globaly = sin(pose[2]) * BW2FW + cos(pose[2]) * (0) + pose[1];
	pose[0] = globalx;
	pose[1] = globaly;
}

void teensyCallback(const dynamo_msgs::TeensyReadPtr& msg) {
	//vel = msg->speed_wheel;
	teensy_left = msg->distance_left;
	teensy_right = msg->distance_right;

}

void drivePointsCallback (const auto_navi::driveMsg::ConstPtr& msg) {
	if (msg->header.frame_id == "mission_control") {
		x_y_th[0] = msg->x;
		x_y_th[1] = msg->y;
		x_y_th[2] = msg->theta;
	}
}

void drive() { //this has been modified to only work on forward locomotion with ackermann steering
	dynamo_msgs::SteeringStepper steeringmsg;
	double angleerr, dvel, d, u, dlim, refthl, refxl, refyl, th, kdist_drive, kangle_drive;
	double dist, target_dist, vnx, vny, c, steering_angle;
//	kdist_drive = 0.3; // weight for error in distance (higher kdist compared to kangle will make car go onto line more aggressively)
//	kangle_drive = 1.2; //weight for error in angle  (higher kanle compared to kdist will make car go smoother onto line)

	if (vel > 5.5 && vel < 10.5) {
		kdist_drive = 0.2;
		kangle_drive = 0.6;
	} else if (vel >= 10.5 && vel < 14.5) {
		kdist_drive = 0.025;  // 0.1
		kangle_drive = 0.5;  // 0.4
	} else if (vel >= 14.5 && vel < 20.5) {
		kdist_drive = 0.01;  // 0.1
		kangle_drive = 0.3;  // 0.4
	} else if (vel >= 20.5) {
		kdist_drive = 0.005;  // 0.1
		kangle_drive = 0.1;  // 0.4
	}

	else {
		kdist_drive = 0.2;
		kangle_drive = 0.6;
	}

	refxl = x_y_th[0];
	refyl = x_y_th[1];
	refthl = x_y_th[2];

	refxl = x_y_th[0];
	refyl = x_y_th[1];
	refthl = x_y_th[2];

	vnx = -sin(refthl); //normal vector
	vny = cos(refthl);
	c = -(vnx * (refxl) + vny * (refyl)); //c = b*y + a*x

	// controller
	angleerr = refthl - pose[2]; //error between current th and target th
	int factor = abs(angleerr / M_PI);
	if (angleerr > M_PI) angleerr = angleerr - factor * M_PI;
	if (angleerr < M_PI) angleerr = angleerr + factor * M_PI;
	d = (pose[0] * vnx + pose[1] * vny + c) ; //steering dist error


	if (kdist_drive > 0) { //limit if target is far away (to not drive in circle)
		dlim = fabs(kangle_drive * M_PI / 2 / kdist_drive); //((0.1+vel)*2)
		if (fabs(d) > dlim) {
			d = dlim * boost::math::sign(d);
		}
	}

	//ACKERMANN-STEERING
	u = -kdist_drive * d + kangle_drive * angleerr; //steering angle

	for (int i = N - 1; i > 1; i--) {
		anglevec[i] = anglevec[i - 1];
	}
	anglevec[0] = u;
	for (int i = 0; i < N; i++) {
		u += anglevec[i] / N;
	}

	//printf("Steering angle is: %f\n",u);
	//hardcode limits or set limits directly in steering wheel program?
	if (sim == 0) u = u * 180.0 / M_PI; //   limit(u, lim, -lim); //steering angle limited by wheel limits
	if (sim && u > limit) u = limit;
	if (sim && u < -limit) u = -limit;
	if (sim && u > last_u + d_u) u = last_u + d_u;
	if (sim && u < last_u - d_u) u = last_u - d_u;
	if (sim) last_u = u;
	steeringmsg.steering_angle = u;      //   limit(u, lim, -lim); //steering angle limited by wheel limits
	steeringmsg.steering_stepper_engaged = 1;
	steerPub.publish(steeringmsg);

	printf("\n\n****NEW RUN ****\nDrive line: x: %f y: %f th: %f.\nPosition: x: %f y: %f th: %f.\nAngle: %f.\nDistance error: %f\nAngle error: %f\n Vel: %f\n", x_y_th[0], x_y_th[1], x_y_th[2], pose[0], pose[1], pose[2], u, d, angleerr, vel);
}

void parking_drive() { //this has been modified to only work on forward locomotion with ackermann steering
	dynamo_msgs::SteeringStepper steeringmsg;
	double angleerr, dvel, d, u, dlim, refthl, refxl, refyl, th, kdist_parking_drive, kangle_parking_drive;
	double dist, target_dist, vnx, vny, c, steering_angle;
	kdist_parking_drive = 0.3; // weight for error in distance (higher kdist compared to kangle will make car go onto line more aggressively)
	kangle_parking_drive = 1.2; //weight for error in angle  (higher kanle compared to kdist will make car go smoother onto line)

	refxl = x_y_th[0];
	refyl = x_y_th[1];
	refthl = x_y_th[2];

	vnx = -sin(refthl); //normal vector
	vny = cos(refthl);
	c = -(vnx * (refxl) + vny * (refyl)); //c = b*y + a*x


	// controller
	angleerr = refthl - pose[2]; //error between current th and target th
	int factor = abs(angleerr / M_PI);
	if (angleerr > M_PI) angleerr = angleerr - factor * M_PI;
	if (angleerr < M_PI) angleerr = angleerr + factor * M_PI;
	d = (pose[0] * vnx + pose[1] * vny + c) ; //steering dist error


	if (kdist_parking_drive > 0) { //limit if target is far away (to not drive in circle)
		dlim = fabs(kangle_parking_drive * M_PI / 2 / kdist_parking_drive); //((0.1+vel)*2)
		if (fabs(d) > dlim) {
			d = dlim * boost::math::sign(d);
		}
	}

	//ACKERMANN-STEERING
	u = -kdist_parking_drive * d + kangle_parking_drive * angleerr; //steering angle

	for (int i = N - 1; i > 1; i--) {
		anglevec[i] = anglevec[i - 1];
	}
	anglevec[0] = u;
	for (int i = 0; i < N; i++) {
		u += anglevec[i] / N;
	}

	//hardcode limits or set limits directly in steering wheel program?
	if (sim == 0) u = u * 180.0 / M_PI; //   limit(u, lim, -lim); //steering angle limited by wheel limits
	if (sim && u > limit) u = limit;
	if (sim && u < -limit) u = -limit;
	if (sim && u > last_u + d_u) u = last_u + d_u;
	if (sim && u < last_u - d_u) u = last_u - d_u;
	if (sim) last_u = u;
	steeringmsg.steering_angle = u;      //   limit(u, lim, -lim); //steering angle limited by wheel limits
	steeringmsg.steering_stepper_engaged = 1;
	steerPub.publish(steeringmsg);

	printf("\n\n****PARKING MODE ****\nDrive line: x: %f y: %f th: %f.\nPosition: x: %f y: %f th: %f.\nAngle: %f.\nDistance error: %f\nAngle error: %f\nVel: %f\n", x_y_th[0], x_y_th[1], x_y_th[2], pose[0], pose[1], pose[2], u, d, angleerr, vel);
}


void emergencyCallback(const auto_navi::emergencyMsg::ConstPtr& msg) {
	master_aktivator = msg->master_switch;
	ROS_WARN("received shutdown");
	if (master_aktivator == false) ros::shutdown();
}


void speed() {		// NOT TO USE IN REAL CAR, ONLY IN SIMULATION
	/*
	int burn = 0;
	if (state == 1 && vel > MAXSPEED) state = 2; //over maxspeed
	if (state == 2 && vel < MINSPEED) state = 1; //under minspeed
	if (state == 1) burn = 1;
	if (state == 2 || state == 3) burn = 0;
	dynamo_msgs::TeensyWrite teensyMsg;
	teensyMsg.burn = burn;
	teensyPub.publish(teensyMsg);
	*/
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "linedrive");
	state = 1;
	ros::NodeHandle n;

	ros::Subscriber sub_pose_rosbag = n.subscribe("car_pose_estimate", 10, rosbagCallback);
	ros::Subscriber sub_drive_points = n.subscribe("drive_points", 10, drivePointsCallback);
	ros::Subscriber sub = n.subscribe("cmd_master_switch", 10, emergencyCallback);
	ros::Subscriber teensySub = n.subscribe("teensy_read", 1, teensyCallback);

	//teensyPub = n.advertise<dynamo_msgs::TeensyWrite>("teensy_write", 100);
	steerPub = n.advertise<dynamo_msgs::SteeringStepper>("cmd_steering_angle", 100);
	float loopRate = 100; // loop rate in [Hz]
	ros::Rate r(loopRate);
	//ros::Publisher drive_publish = n.advertise<std_msgs::double>("steering_angle",10);
	double steering_angle;

	while (ros::ok()) {
		ros::spinOnce();
		speed();
		if (parking) {
			parking_drive();
		} else {
			drive();
		}
		// Construction of the Voronoi Diagram.

		r.sleep();

	}

	ROS_INFO("below ros ok");
	return 0;
}

