#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <stdlib.h>
#include <fstream>

using namespace std;

#define LOGFILE_NAME "/home/dynamo/log_files/RosbagLog_%d.txt"
#define ANGLES_2_RADIANS 0.0174533

double pose[3] = {0, 0, 0};

void rosbagCallback (const std_msgs::Float32MultiArray::ConstPtr& msg_pose_est) {
pose[0] = msg_pose_est->data[0];
pose[1] = msg_pose_est->data[1];
pose[2] = msg_pose_est->data[2];
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "pose_rosbag_subscriber");

	ros::NodeHandle n;

	ros::Subscriber sub_pose_rosbag = n.subscribe("car_pose_estimate", 1, rosbagCallback);

	float loopRate = 50; // loop rate in [Hz]
	ros::Rate r(loopRate);


	char filename[100];
	sprintf(filename, LOGFILE_NAME, (int)ros::WallTime::now().toSec() - 1494796385);
	ofstream logFile(filename, ios::out | ios::binary);
	logFile << "Extracted from .bag file";
	logFile << "Time,Estimate x,Estimate y,Estimate theta\n";

	//logFile.close();

	while (ros::ok()) {
		
		ros::spinOnce();

		logFile << ros::Time::now() << "," <<  pose[0] << "," << pose[1] << "," << 			pose[2] << "\n";

		r.sleep();
	}
	logFile.close();
	return 0;
}
