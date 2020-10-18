#ifndef _BRAKESYSTEM_SOURCE_
#define _BRAKESYSTEM_SOURCE_
#include "BrakeSystem.h"
#include <iostream>


void BrakeSystem::BrakePID(float target)
{
	float press, Kp, Ki;
	/* By default use sensor press2 (rear wheels) but if press1 is at least 10% bigger use it instead.
	The rear wheels are preferred because when the Ecocar is without cover shell then only
	the rear sensor is working.
	*/
	if (press1 > press2*1.1) // The 10% margin is to avoid jumping from one sensor to the other if values are close
	{
		ROS_INFO("Using press 1\n");
		press = press1;
	}
	else
	{
		ROS_INFO("Using press 2\n");
		press = press2;
	}
	ROS_INFO("Pressure: %f\n",press);
	// PI gains
	Kp = KC;
	Ki = IC;
	
	/*
	// Use different PI gains depending on pressure level
	if (press > Gain_sched)
	{
		ROS_INFO("Gain 1\n");
		Kp = KC;
		Ki = IC;
	}
	else
	{
		ROS_INFO("Gain 0\n");
		Kp = KC0;
		Ki = IC0;
	}*/

	// Calculate error
	float error = target - press;
	ROS_INFO("Control error: %f\n",error);
	// P gain
	float Kout = Kp * error;
	// I gain
	integral += error * tConst;
	float Iout = Ki * integral;
	// D gain
	/*derivate = (error - pre_error) / tConst; // Not going to work like this without digital low-pass filter
	float Dout = DC * derivate; */
	
	// Output of PI
	//float output = Kout + Iout + Dout;
	float output = Kout + Iout; // Voltage for Phidgets DAC that drives the Pololu position controller
	
	// Anti-wind up
	if (output > posMax || output < posMin) 
	{
		ROS_INFO("Warning anti windup!\n");
		Iout = Ki * pre_integral; // Recalculate Iout with previous value if output is saturated
		integral = pre_integral; // Freeze integral
		output = Kout + Iout;
	}
	
	ROS_INFO("Integrator: %f\n",Iout);
	ROS_INFO("Integrator_raw: %f\n",integral);

	// Output saturation
	if (output > posMax)
	{
		ROS_INFO("Warning saturation!\n");
		output = posMax;
	}
	else if (output < posMin)
	{
		output = posMin;
	}

	pre_error = error; // Set error as previous error
	pre_integral = integral; // Update integrator (required for anti-wind up)

	position = output; // Position voltage

}

void BrakeSystem::SentToCon(double Vol)
{
	jrkG2.setTarget(Vol);
}

void BrakeSystem::UpdateRef(float brakePot, float brake_press_1, float brake_press_2)
{
	pot = (5/1023)*brakePot; // Position of the linear actuator in voltage scaled to 10-bit
	
	/* It seems that brake_press_1 and brake_press_2 are not the raw ADC values
	but they're already converted into bar previously!
	- Sensor output is between 1 to 5V with operative range from -1 to 34 bar
	- Scaling is: bar = -39/4+35/4*voltage
	- With 10-bit ADC (raw value from 0 to 1023): bar = -9.75 + 0.042766373411535 * raw
	*/
	//press1 = 0.001732418442*brake_press_1-11; // Calculate ADC value to Bar
	//press2 = 0.001732418442*brake_press_2-11;
	press1 = brake_press_1;
	press2 = brake_press_2;
	brakepres = brake_press_2; // Seems unused
}

bool BrakeSystem::CalibrateBrake() // Calibration commented out because not required anymore (done through Pololu controller if required)
{/*
	if (firstRun == true)
	{
		position = 0;
		potDif = pot - pre_pot;
		if(timers > 10)
		{
			if(potDif == 0)
			{
				firstRun = false;
				posMin = pot;
			}	
			timers = 0;
		}
		pre_pot = pot;
	}
	else
	{
		if(press2 <= Max_Pressure)
		{
			position = position + 0.005;
			return false;
		}
		else
		{
			posMax = pot;
			return true;
		}
	}*/
}

BrakeSystem::BrakeSystem(float time)
{
	tConst = 1/time;
	pre_error = 0;
	integral = 0;
	derivate = 0;
	firstRun = true;
	pre_pot = 0;
	potDif = 1;
	posMin = 0;
	position = 0;

	jrkG2.connect();

}

void BrakeSystem::close(){

}

#endif // !_BRAKESYSTEM_SOURCE_
