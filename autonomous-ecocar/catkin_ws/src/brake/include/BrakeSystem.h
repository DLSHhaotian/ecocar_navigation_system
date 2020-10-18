#ifndef _BRAKESYSTEM_H_
#define _BRAKESYSTEM_H_


#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <stdio.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
//#include <serial_tools.h>
#include <dynamo_msgs/TeensyWrite.h>
#include <dynamo_msgs/TeensyRead.h>
#include <dynamo_msgs/BrakeStepper.h>
#include <fstream>
//#include <phidget21.h>
#include "JrkG2.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#define Min_Pressure 0
#define Max_Pressure 34
#define KC 0.1 // This would be K_p in the parallel form of a PI --> G(s) = K_p + K_i/s
#define IC 0.5 // This is K_i
#define DC 0 // Disregarded --> PI only
/*
#define KC0 0.1 // Gain scheduling: this is the more aggressive P-gain to be used only at low pressures
#define IC0 0.5 // Gain scheduling: this is the more aggressive I-gain to be used only at low pressures
#define Gain_sched 0.1 // Pressure level in bar where the controller gains change from KC0 to KC and from IC0 to IC
// If the gain scheduling is causing problems then just put KC0 the same as KC and IC0 the same as IC to disable it!
*/

//#define ANALOG_PHIDGET_SERIAL 493613
//#define WAIT_FOR_ATTACH_TIMEOUT 5000 

class BrakeSystem
{
	//CPhidgetAnalogHandle phid;
	JrkG2 jrkG2;
	float pre_error;
	float derivate;
	float tConst = 1/20; // Sample time (frequency is defined in BrakeSystem.cpp --> CONTROL_LOOP_RATE 20)
	bool firstRun;
	float pre_pot;
	float potDif;

public:	
	BrakeSystem(float time);
	void BrakePID(float target); // Input Target value and Current Value for calculating the error. Returns Control signal
	void UpdateRef(float brakePot, float brake_press_1, float brake_press_2);
	bool CalibrateBrake();
	void SentToCon(double Vol);
	void close();

	float posMax = 5; /* Position voltage. Calibration of this value is not required anymore because position limitation 
	and max current limitation to prevent destruction can both be done directly on the Pololu controller 
	of the linear actuator by connecting it through USB
	HOWEVER, NEVER SET LIMIT ABOVE 5V BECAUSE IT WILL DAMAGE THE POLOLU CONTROLLER!!
	*/
	float posMin = 0;
	float pot;
	float press1;
	float press2;
	float integral;
	float pre_integral; // Added to implement integral anti-windup
	float power;
	float position;
	ros::Time timenow;
	ros::Time pre_time;
	bool engage;
	double timers = 0;
	float brakepres;
};

#endif // !_BRAKESYSTEM_H_
