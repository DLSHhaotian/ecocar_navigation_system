//
//  steeringAngle.hpp
//  steeringAngle
//
//  Created by Alessandra Bellina on 24/04/2017.
//  Copyright © 2017 Alessandra Bellina. All rights reserved.
//

#ifndef steeringAngle_hpp
#define steeringAngle_hpp

#include <stdio.h>
#include <iostream>

//Position class declaration
class Position
{
public:
    double x, y, theta;
    //Constructor:
    Position();
    Position (double xValue, double yValue, double thetaValue);
    
};

class Goal
{
public:
    double x, y, speed, tolerance;
    //Constructor:
    Goal();
    Goal (double xValue, double yValue, double thetaValue, double wantedsPeed, double wantedTolerance);
    
};


//Line class declaration
class Line
{
public:
    double m, q;
    //Constructor:
    Line();
    Line(double mValue, double qValue);
};



const double TS = 0.1;
//const double SATURATION_LIMIT = 0.5;
const double DISTANCE_TO_LINE = 3;  //remember to add SPEED/10 inside the function
const double gain = 10;
const int NUMBER_OF_PREDICTIONS=10;
/* it's in steeringAngle.cpp
	//Define goals:
	const Goal goal0(0,0,0,0,0);
	const Goal goal1(30,0,0,1,0.5);
	const Goal goal2(50,10,0,2,0.5);
	const Goal goal3(60,20,0,2,0.5);
	const Goal goal4(70,40,0,2,0.5);
	const Goal goal5(60,60,0,2,0.5);
	const Goal goal6(50,70,0,1,0.5);
	const Goal goal7(20,80,0,1,0.5);
	const Goal goal8(20,80,0,0,0);
	Goal goalsArray [9] = {goal0, goal1,goal2, goal3, goal4, goal5,goal6,goal7,goal8};// remember to fix this!!!!
	const int numberOfGoals = sizeof(goalsArray) / sizeof(goalsArray[0]);
*/

/*
const Line line1(0,0);
const Line line2(0.5,-1.5);
const Line line3(1,-4);
const Line line4(2,-10);
const Line line5(-2,18);
const Line line6(-1,12);
const Line line7(-1/3,8.66666666666666666);
const Line lines [7] = {line1, line2,line3, line4, line5, line6,line7}; // remember to fix this!!!!
int number_of_lines = sizeof(lines)/sizeof(lines[0]);
*/
const double pi = 3.141592653589793238462643383279502884;




//updateGoalOutput class declaration
class updateGoalOutput
{
public:
    Goal goal; int currentLine; bool backOnLine;
    //Constructor:
    updateGoalOutput ();
    updateGoalOutput (Goal newGoal, int newCurrentLine, bool newBackOnLine);
    
};


//findsolOutput class declaration
class findsolOutput
{
public:
    Goal solution; bool needToChangeLine;
    //Constructor:
    findsolOutput ();
    findsolOutput (Goal sol, bool needToChange);
};

// This function returns the steering angle, given the current position of
// the car and the goal position. It calculates the steering angle, predict the movement and calculate the new steering angle for i-times. The output is the average of the calculated steering angles.
double getSteeringAngle(Position currentPosition, int currentLine, Goal goal, bool backOnLine, double speed, Goal goalsArray[], Line lines[]);

// These function are used by getSteeringAngle
double calculateSteeringAngle(Position currentPosition, Goal goal, double speed); // returns the steering angle

Position predictMovement(double steeringAngle, Position currentPosition, double speed); //modifies the position and the real steering angle

// This function calculates the appropriate goal, to be used with getSteeringAngle. It modifies the goal
updateGoalOutput updateGoal(Position currentPosition,  int currentLine, double speed, bool backOnLine, Goal goalsArray[], Line lines[]);

//this function is used by update goal
findsolOutput findsol(Position currentPosition, int currentLine, double distance, Goal goalsArray[], Line lines[]);

//steeringAngleOutput class declaration
class steeringAngleOutput
{
public:
    double angle;int currentLine; bool goingBack; bool accelerate; float brake;
    //Constructor:
    steeringAngleOutput ();
    steeringAngleOutput (double angle, int currentLine, bool goingBack, bool acc, float brk);
};

steeringAngleOutput steeringAngle(Position currentPosition, int currentLine, bool backOnLine, double speed, Goal goalsArray[], Line lines[]);

void createLines(const Goal goalsArray[], Line lines[], int sizeofGoalsArray);






#endif /* steeringAngle_hpp */
