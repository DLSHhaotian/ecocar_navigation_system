#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include "auto_navi/driveMsg.h"
#include "dynamo_msgs/SteeringStepper.h"
#include "dynamo_msgs/BrakeStepper.h"
#include "auto_navi/emergencyMsg.h"
#include "dynamo_msgs/TeensyWrite.h"
#include "geometry_msgs/PolygonStamped.h"
#include "dynamo_msgs/TeensyRead.h"
#include "lidar_package/gates.h"

// ROS stuff
ros::Publisher voroPub;
ros::Publisher steerPub;
ros::Publisher teensyPub;
ros::Publisher marker_pub;
// Variables
double odox, odoy, odotheta;	// odometry from estimate_pose (Teensy)
double x = 0, y = 0, th = 0;		// the drive point at parking spot
double speed, drivendist;	// from Teensy'
double starting_drivendist, pointdist;
double BW2FW = 1.516;
int send_the_point = 0;
double dest = 0;

void carPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_carPoseEstimate) {
	// store vlaues from odometry
	odox = msg_carPoseEstimate->data[0];
	odoy = msg_carPoseEstimate->data[1];
	odotheta = msg_carPoseEstimate->data[3];

}


void teensyCallback(const dynamo_msgs::TeensyReadPtr& msg) {
	speed = msg->speed_wheel;
	drivendist = msg->distance_wheel;
}


void point_sender() {
	auto_navi::driveMsg messy;

	// Initilize drive point message
	messy.header.stamp = ros::Time::now();
	messy.header.frame_id = "gate_node";


	// the points are already global?
	double globalx = x;
	double globaly = y;

	//th = odotheta+th;
	//if(globalx==0 || drivendist <= dest) return;
	// publish (send to drive)
	dest = drivendist + sqrt(pow(x - odox, 2) + pow(y - odoy, 2));
	messy.x = globalx;
	messy.y = globaly;
	messy.theta = th;
	voroPub.publish(messy);
}




void rvizzer2 () {


	uint32_t shape = visualization_msgs::Marker::POINTS;
	visualization_msgs::Marker points_line;
	ros::Duration one_sec(.4);

	points_line.header.frame_id = "visualization_frame";//"base_link";
	points_line.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	points_line.ns = "points_line";

	points_line.action = visualization_msgs::Marker::ADD;
	points_line.pose.orientation.w = 1.0;


	points_line.id = 1;

	// Set the marker type.

	points_line.type = visualization_msgs::Marker::POINTS;



	points_line.scale.x = 0.2;
	points_line.scale.y = 0.2;

	points_line.color.g = 1.0f;	// green
	points_line.color.a = 1.0;



	// points_line
	int amount = 15;
	double resol = 0.25;
	for (int i = 0; i < amount; i++) {
		geometry_msgs::Point p2;
		p2.x = x + cos(th) * resol * i;
		p2.y = y + sin(th) * resol * i;
		p2.z = 0;
		points_line.points.push_back(p2);
	}




	marker_pub.publish(points_line);
}


void gates(const  lidar_package::gates::ConstPtr& msg) {
	// this node receives message from gate node
	double x_c = 0.0, x_c_temp, y_c, y_c_temp, x_col1, y_col1, x_col2, y_col2;
	//calculate drive point and theta
	double pointdist_closest = 100;
	double pointdist_new;
	double dist1, dist2;
	int opt = 0;
//	printf("vec_len:%d\n",msg->vector_len);
	for (int i = 0; i < msg->vector_len; i++) {

		x_c_temp = msg->vector_gates[i].center.x;
		//if(x_c_temp < odox ) continue;
		y_c_temp = msg->vector_gates[i].center.y;
		pointdist_new = sqrt(pow(x_c_temp - odox, 2) + pow(y_c_temp - odoy, 2));
//		printf("x_c: %f , y_c: %f\n",x_c_temp,y_c_temp);

		if (pointdist_closest > pointdist_new) {
			pointdist_closest = pointdist_new;

			x_c = x_c_temp;
			y_c = y_c_temp;
			x_col1 = msg->vector_gates[i].column_1.x;
			y_col1 = msg->vector_gates[i].column_1.y;
			x_col2 = msg->vector_gates[i].column_2.x;
			y_col2 = msg->vector_gates[i].column_2.y;

//			printf("center: (%f,%f), col1: (%f,%f) col2: (%f,%f)\n",x_c,y_c,x_col1,y_col1,x_col2,y_col2);
			if (fabs(y_col1) >= fabs(y_col2)) opt = 1;
			if (fabs(y_col2) > fabs(y_col1)) opt = 2;

		}


	}

	if (x_c == 0.0) return;
	double th_temp;

	if (opt == 1) {
		th_temp = atan2(y_col1 - y_c, x_col1 - x_c);
	} else {
		th_temp = atan2(y_col2 - y_c, x_col2 - x_c);
	}

	if (fabs(th_temp - M_PI / 2) > fabs(th_temp + M_PI / 2)) {
		th_temp += M_PI / 2;
	} else {
		th_temp -= M_PI / 2;
	}
	x = x_c;
	y = y_c;
	th = th_temp;


	point_sender();
	ROS_INFO("Received point: x = %f  y = %f  theta = %f, vector_len: %d", x, y, th, msg->vector_len);

	rvizzer2();

}


int main(int argc, char **argv) {
	// Initialization
	ros::init(argc, argv, "parking_node");
	ros::NodeHandle n;
	pointdist = 90.0;

	ros::Subscriber odo_sub = n.subscribe("car_pose_estimate", 1000, carPoseCallback);
	ros::Subscriber teensy_sub = n.subscribe("teensy_read", 10, teensyCallback);
	ros::Subscriber gates_sub = n.subscribe("gates", 0, gates);

	steerPub = n.advertise<dynamo_msgs::SteeringStepper>("cmd_steering_angle", 100);
	teensyPub = n.advertise<dynamo_msgs::TeensyWrite>("teensy_write", 100);
	voroPub = n.advertise<auto_navi::driveMsg>("drive_points", 1);
	marker_pub = n.advertise<visualization_msgs::Marker>("gates_line", 0);

	float loopRate = 20; // loop rate in [Hz]
	ros::Rate r(loopRate);

	while (ros::ok()) {
		ros::spinOnce();

		//sim_parker();	// only in simulation

		r.sleep();

	}

//  ros::spin();

	std::cout << "\n\n\nROS no longer ok \n\n\n";
	return 0;
}
