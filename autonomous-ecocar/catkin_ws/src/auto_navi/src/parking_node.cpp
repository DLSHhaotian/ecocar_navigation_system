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

// Time
ros::Time t1;
ros::Time t2;
ros::Duration t_diff;
// ROS stuff
ros::Publisher voroPub;
ros::Publisher marker_pub;
ros::Publisher brake_pub;
ros::Publisher steerPub;
ros::Publisher teensyPub;
ros::Publisher emergency_publisher;
// Variables
double odox, odoy, odotheta;	// odometry from estimate_pose (Teensy)
double x,y, th;		// the drive point at parking spot
double speed, drivendist;	// from Teensy'
double starting_drivendist, pointdist, x_0, x_1, y_0, y_1;
double BW2FW=1.516;
double max_brake_power = 0.0;
int first = 1;
int send_the_point = 0;
geometry_msgs::Point32 p_front;
geometry_msgs::Point32 p_center;

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


void point_sender(){
	auto_navi::driveMsg messy;
	
	// calculate points to global coordinates
	//double globalx = cos(odotheta)*x-sin(odotheta)*y+odox;
	//double globaly = sin(odotheta)*x+cos(odotheta)*y+odoy;

	// the points are already global?
	double globalx = x;
	double globaly = y;
	
	//th = odotheta+th;	

	// publish (send to drive)
	messy.x = globalx;
	messy.y = globaly;
	messy.theta = th;
	voroPub.publish(messy);	
}



void shutter() {

	// brake hard

	dynamo_msgs::BrakeStepper bremse;
	bremse.brake_stepper_engaged = true;
	bremse.brake_power = max_brake_power;
	brake_pub.publish(bremse);

}

void rvizzer2 () {

	uint32_t shape = visualization_msgs::Marker::POINTS;
    visualization_msgs::Marker points;
	ros::Duration one_sec(.1);	// .4?

	points.header.frame_id = "visualization_frame";//"base_link";
    points.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
     points.ns = "parking_points";
	
     points.action = visualization_msgs::Marker::ADD;
     points.pose.orientation.w = 1.0;
	
     points.id = 0;
    
    // Set the marker type.
  
    points.type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // Points are blue
    points.color.b = 1.0;
    points.color.a = 1.0;

	// push back
	geometry_msgs::Point pc;
	geometry_msgs::Point pf;
	
	//pc.x = p_center.x;	// real world
	//pc.y = p_center.y;
	//pf.x = p_front.x;
	//pf.y = p_front.y;
	pc.x = x_1;				// sim
	pc.y = y_1;
	pf.x = x_0;
	pf.y = y_0;
	
	points.points.push_back(pc);
	points.points.push_back(pf);
	
    points.lifetime =one_sec;
    marker_pub.publish(points);

}

void parker(const geometry_msgs::PolygonStamped::ConstPtr& msg) {
	// this node receives message from camera node

	if (first == 1) {
	//calculate drive point and theta
		p_center = msg->polygon.points[0];	// center of parking spot
		p_front = msg->polygon.points[1];	// center of front line of parking spot
		y_1 = p_center.y;
		x_1 = p_center.x;
		y_0 = p_front.y;
		x_0 = p_front.x;
		th = atan2(y_1-y_0,x_1-x_0);
		x = x_1;
		y = y_1;
		
		//if(msg->polygon.points[0].x != 0) 
		send_the_point = 1;
		ROS_INFO("Received point: x = %f  y = %f  theta = %f",x,y,th);
		first = 0;
	}
	
	if (send_the_point == 1) {
		point_sender();
		
		double odo_front_wheel_x = cos(odotheta)*BW2FW-sin(odotheta)*0+odox;
		double odo_front_wheel_y = sin(odotheta)*BW2FW+cos(odotheta)*0+odoy;
		pointdist = sqrt(pow(x-odo_front_wheel_x,2)+pow(y-odo_front_wheel_y,2));
		ROS_INFO("Pointdist: %f", pointdist);
	}
	
	if (pointdist < 0.2 && send_the_point ==1) { // brake accordin to position 
		// brake and cut motor + steering
		//shutter();
		send_the_point = 0;
		ROS_INFO("Brake, since pointdist: %f",pointdist);
	}
	
	
}





int main(int argc, char **argv) {
	// Initialization
	ros::init(argc, argv, "parking_node");
	ros::NodeHandle n;
	t1 = ros::Time::now();	// time at node start
	pointdist = 90.0;
	
    	ros::Subscriber odo_sub = n.subscribe("car_pose_estimate", 1000, carPoseCallback);
	ros::Subscriber parking_sub = n.subscribe("parking_spot_detection/corners", 10, parker);
	ros::Subscriber teensy_sub = n.subscribe("teensy_read", 10, teensyCallback);
	
	brake_pub = n.advertise<dynamo_msgs::BrakeStepper>("cmd_brake_power",10);
	steerPub = n.advertise<dynamo_msgs::SteeringStepper>("cmd_steering_angle",100);
	emergency_publisher = n.advertise<auto_navi::emergencyMsg>("cmd_master_switch",10);
	teensyPub = n.advertise<dynamo_msgs::TeensyWrite>("teensy_write", 100); 
	voroPub = n.advertise<auto_navi::driveMsg>("drive_points",1);
	marker_pub = n.advertise<visualization_msgs::Marker>("parking_marker", 0);
	
	
	float loopRate = 20; // loop rate in [Hz]
	ros::Rate r(loopRate);
	
	while (ros::ok()) {		
		ros::spinOnce();
		
		//sim_parker();	// only in simulation
		
		r.sleep();
		
	}

//  ros::spin();

	std::cout<<"\n\n\nROS no longer ok \n\n\n";
	return 0;
}
