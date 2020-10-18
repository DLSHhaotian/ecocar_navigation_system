#include "ros/ros.h"
#include "navigation/global_variables.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "LocalCartesian.hpp"
#include "LocalCartesian.cpp"
#include <stdlib.h>
#include <fstream>
#include <dynamo_msgs/TeensyRead.h>
#include <dynamo_msgs/fusion.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include <EFK.hpp>
#include <EFK.cpp>

#define DEG2RAD 0.0174533

#define SINGLE_POINT_SIGMA 3.0
#define DGPS_SIGMA 2.0
#define FLOAT_SIGMA 0.5
#define FIXED_SIGMA 0.1

//YEAAAAH WE LOVE LOGFILES
#define LOGFILE_NAME "/home/dynamo/log_files/fusionLog_%d.txt"

//Filter initial parameters:
//Process noise covariance
double Q[n][n] = {
        {0.0001,      0,      0,      0},
        {     0, 0.0001,      0,      0},
        {     0,      0, 0.0001,      0},
        {     0,      0,      0, 0.0001}};

//Meaasurement noise covariance
double R[m][m] = {
        {0.04,         0},
        {   0, 0.000001}};

//Initial state
double x[n] = {0;0;0;0};

//Initial uncertainty
double P[n][n] = {
        {0.25,      0,                           0,    0},
        {     0, 0.25,                           0,    0},
        {     0,    0, 20 * 20 * DEG2RAD * DEG2RAD,    0},
        {     0,    0,                           0, 0.01}};

double z[m]; //predicted output


float messageOdoSpeed = 0;
float messageGyroAngle = 0;
double OdoEstimate[3];
float GPSAccuracy;
double headingReference = 0.0;
double latRef = 0.0;
double lonRef = 0.0;
double hRef = 0.0;
PointCoordinates PRef;
PointCoordinates GPSEstimate;

double streamTimeOdo = 1;
double streamTimeGPS = 1;
bool gotNewGPSReading = 0;

//If we put this callback first, will it spin through it first?
void headingCallback (const std_msgs::Float32::ConstPtr& msg_heading) {
	headingReference = msg_heading->data;
	headingReferenceSubscriber.shutdown() //shut down the subscriber when reference received
}

// General TODO: subscribe to steering angle from Henning

void odoVelocityCallback (const dynamo_msgs::TeensyRead::ConstPtr& msg_odo_speed) {
	streamTimeOdo = 0;
	previousOdo = messageOdoSpeed;
	messageOdoSpeed = msg_odo_dist->speed_wheel;
	odoReadingTime = ros::Time::now().toSec();
}

void gyroAngleCallback (const std_msgs::Float32::ConstPtr& msg_gyro_angle) {
	streamTimeGyroAngle = 0;
	messageGyroAngle = (msg_gyro_angle->data) * ANGLES_2_RADIANS;
}


void GPSPoseCallback (const sensor_msgs::NavSatFixPtr::ConstPtr& msg_GPS) {
	streamTimeGPS = 0; 
	gotNewGPSReading = true;

	Pointcoordinates P0_GPS;
	P0_GPS.c1 = msg_GPS->latitude;
	P0_GPS.c2 = msg_GPS->longtitude;
	P0_GPS.c3 = msg_GPS->altitude;

	//Estimate GPS accuracy out of mode
	if ( msg_GPS->status.status == 1) GPSAccuracy = SINGLE_POINT_SIGMA;
	if ( msg_GPS->status.status == 2) GPSAccuracy = DGPS_SIGMA;
	if ( msg_GPS->status.status == 3) GPSAccuracy = FLOAT_SIGMA;
	if ( msg_GPS->status.status == 4) GPSAccuracy = FIXED_SIGMA;

	
	//Convert readings to local frame
	GPSEstimate = GPS_2_LOCAL(P0_GPS, PRef, headingReference);

	ROS_INFO("Obtained GPS point coordinates : [%f %f %f], accuracy [%f]"GPSEstimate.c1,
   GPSEstimate.c2, GPSEstimate.c3, GPSAccuracy);

}

void EKFIterate () {

  if (gotNewGPSReading == true) { // if a GPS signal obtained fuse it
    float x1 = x[0];
    double varx1 = P[0][0];
    float y1 = x[1];
    double vary1 = P[1][1];

	 float xGPS = GPSEstimate.c1;
	 float yGPS = GPSEstimate.c2;
    double outputvarx = 1/(1/ varx1+1/ (double)GPSAccuracy);
    double outputvary = 1/(1/ vary1+1/ (double)GPSAccuracy);
    double outputx = outputvarx*(x1/varx1 + xGPS/(double)GPSAccuracy);
    double outputy = outputvary*(y1/vary1 + yGPS/(double)GPSAccuracy); 

    x[0] = outputx;
	 x[1] = outputy;
	 
	 P = {
        {outputvarx,          0,                           0,        0},
        {         0, outputvary,                           0,        0},
        {         0,         0, 0.1 * 0.1 *DEG2RAD * DEG2RAD,        0},
        {         0,         0,                            0, 0.000001}};

	gotNewGPSReading = false;
  }

  //Update measurement:
  z = {messageOdoSpeed, messageGyroAngle};

		//TODO: make a function for linear matices

	    for(int k = 0; k<n; k++)    //create linear version of P
    {
        for(int i = 0; i<n;i++)
        {
            Plin[i+k*n] = P[k][i];
        }
    }

	    for(int k = 0; k<n; k++)    //create linear version of Q
    {
        for(int i = 0; i<n;i++)
        {
            Qlin[i+k*n] = Q[k][i];
        }
    }

	    for(int k = 0; k<n; k++)    //create linear version of R
    {
        for(int i = 0; i<n;i++)
        {
            Rlin[i+k*n] = R[k][i];
        }
    }


	double Pest[4][4];
	//Overwriting output to have input value - state and state covariance

			for(int k = 0; k < n; ++k)
				for(int i = 0; i < n; ++i) {
				Pestl[i+k*n]=P[k][i];
				}
  	double est[n];
 	for(int k = 0; k < n; ++k) est[k] = x[k];

	// Main stuff - Kalman filter
   ekf(x,Plin,z,Ql,Rl,steeringAngle,est,Pestl);
	
	//updating values of state and covariance
    for(int k = 0; k<n; k++)
    {
        for(int i = 0; i<n;i++)
        {
           P[k][i] = Pestl[i+k*n];
        }
    }

 	for(int k = 0; k < n; ++k) x[k] = est[k];     
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "fusion_node");

	ros::NodeHandle n;

	sprintf(filename, LOGFILE_NAME, (int)ros::WallTime::now().toSec() - 1494796385);
	ofstream logFile(filename, ios::out | ios::binary);
	logFile << "Time,Estimate x,Estimate y,Estimate theta, Estimate speed \n";
	char filename[100];
	
	float loopRate = 50; // loop rate in [Hz]
	ros::Rate r(loopRate);
	
	ros::Subscriber sub_vel = n.subscribe("teensy_read", 1, odoVelocityCallback);
	ros::Subscriber sub_gyro_angle = n.subscribe("gyro_angle", 1, gyroAngleCallback);
	ros::Publisher pub_fusion = n.advertise<dynamo_msgs::fusion>("fusion_states", 1);

	while (ros::ok()) {
		ros::spinOnce();

		EKFIterate ();

		fusion fusionData;
		fusionData.x     = x[0];
		fusionData.y     = x[1];
		fusionData.th    = x[2];
		fusionData.speed = x[3];

		logFile << ros::Time::now() << "," <<  fusionData.x << "," << fusionData.y
      << "," << fusionData.th << fusionData.speed <<"\n";

		streamTimeOdo += 1 / loopRate;
		streamTimeGPS += 1 / loopRate;

		if (streamTimeOdo >= 1)
		ROS_INFO("Warning: Odometry streaming stopped!");

		if (streamTimeGPS >= 1)
		ROS_INFO("Warning: Gyro angle streaming stopped!");

		r.sleep();
	}
	logFile.close();
	return 0;
}





