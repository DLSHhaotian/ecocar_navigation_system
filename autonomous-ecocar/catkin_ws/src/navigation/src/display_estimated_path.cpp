#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "std_msgs/Float32MultiArray.h"

// so far haven't found a better way of passing subscriber values into main ROS loop than global variables
double estimatedCarPath[3];

void estimatedPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_carPoseEstimate) {
	for (int i = 0; i < 3; i++) {
		estimatedCarPath[i] = msg_carPoseEstimate->data[i];
	}
	ROS_INFO("Obtained point coordinates : [%f %f]",estimatedCarPath[0], estimatedCarPath[1]);
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "points_and_lines_estimated");
	ros::NodeHandle n;
	ros::Publisher estimatedPathMarker_pub = n.advertise<visualization_msgs::Marker>("estimated_pose_marker", 10);
	ros::Subscriber estimatedPathMarker_sub = n.subscribe("car_pose_estimate", 10, estimatedPoseCallback);
	ros::Rate r(30);

	while (ros::ok()) {
		visualization_msgs::Marker points;
		points.header.frame_id = "/car_global_frame";
		points.header.stamp = ros::Time::now();
		points.ns = "estimated_pose_points";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 1.0;
		points.id = 0;

		points.type = visualization_msgs::Marker::POINTS; //POINTS

		// POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.2;
		points.scale.y = 0.2;

		// Points are green
		points.color.r = 1.0f;
		points.color.a = 1.0;

		// Create the vertices for the points
		geometry_msgs::Point p;

		p.x = estimatedCarPath[0];
		p.y = estimatedCarPath[1];
		p.z = 0;
		ROS_INFO("Publishing estimated point coordinates : [%f %f]", p.x, p.y);
		points.points.push_back(p);

		estimatedPathMarker_pub.publish(points);
		ros::spinOnce();
		r.sleep();
	}
}
