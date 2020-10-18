#include "ros/ros.h"

#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>

#include "lidar_package/point.h"
#include "lidar_package/obst.h"
#include "lidar_package/obsts.h"
#include "geometry_msgs/Point.h"

// Logfile placement, has to kinda match ~
#define LOGFILE_NAME "dynamo/log_files/pose_log_%d.txt"
#define LOGFILE_PATH "dynamo/log_files"
#define LOGFILE_LOCAL "pose_log_%d.txt"

// Global pose variables
double posex = 0;
double posey = 0;
double posetheta = 0;

void obstacle_hullsCallback(const lidar_package::obsts::ConstPtr& msg) {
	posex = msg->x;  // odometry x
	posey = msg->y;  // odometery y
	posetheta = msg->orientation;  // odometry theta
	return;
}

void log(std::ofstream * file) {
	char logmessage[50];
	sprintf(logmessage, "%f, %f, %f\n", posex, posey, posetheta);
	file->write(logmessage, strlen(logmessage));
	// fprintf (logFile, "%f,%f,%f\n", posex, posey, posetheta);
	return;
}

// Found on the interwebs
int dirExists(char *path) {
	struct stat info;

	if (stat( path, &info ) != 0)
		return 0;
	else if (info.st_mode & S_IFDIR)
		return 1;
	else
		return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "log_estimated_pose");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("obstacle_hulls", 100, obstacle_hullsCallback);

	// Check if the directory exists
	char filename[50];
	if (dirExists(LOGFILE_PATH)) {
		sprintf(filename, LOGFILE_NAME, (int)ros::WallTime::now().toSec() - 1494796385);
	} else {
		ROS_INFO("Saving to local folder, maybe home/.ros");
		sprintf(filename, LOGFILE_LOCAL, (int)ros::WallTime::now().toSec() - 1494796385);
	}

	// Open output file
	std::ofstream file(filename, std::ios::binary);
	// Write top message
	char* intro_message = "Estimate x, Estimate y, Estimate theta\n";
	file.write(intro_message, strlen(intro_message));

	float loopRate = 20; // loop rate in [Hz]
	ros::Rate r(loopRate);
	int count = 0;
	ROS_INFO("Starting main loop.");
	ROS_INFO("Log at %s", filename);
	// Log once per loop
	while (ros::ok())
	{
		ros::spinOnce();
		ROS_INFO("Logging");
		log(&file);
		r.sleep();

		++count;
	}
	file.close();

	return 0;
}