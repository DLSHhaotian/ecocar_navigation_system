#include "ros/ros.h"
#include "navigation/global_variables.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <dynamo_msgs/TeensyRead.h>
#include <dynamo_msgs/TeensyWrite.h>
#include <dynamo_msgs/SteeringStepper.h>
#include <dynamo_msgs/BrakeStepper.h>
#include "navigation/steering_angle.hpp"
#include <valarray>

//Constructor:
Position::Position(double xValue, double yValue, double thetaValue)
{
    x = xValue;
    y = yValue;
    theta = thetaValue;
}

Position::Position()
{
    x = 0;
    y = 0;
    theta = 0;
}


//Constructor:
Goal::Goal(double xValue, double yValue, double thetaValue, double wantedSpeed, double wantedTolerance)
{
    x = xValue;
    y = yValue;
    speed = wantedSpeed;
    tolerance = wantedTolerance;
}

Goal::Goal()
{
    x = 0;
    y = 0;
    speed = 0;
    tolerance = 0;
}

//Constructor:
Line::Line(double mValue, double qValue)
{
    m = mValue;
    q = qValue;
}

Line::Line()
{
    m = 0;
    q = 0;
}

//Constructor for updateGoalOutput:
updateGoalOutput::updateGoalOutput()
{
    goal = Goal();
    currentLine = 0;
    backOnLine = 0;
}

updateGoalOutput::updateGoalOutput(Goal newGoal, int newCurrentLine, bool newBackOnLine)
{
    goal = newGoal;
    currentLine = newCurrentLine;
    backOnLine = newBackOnLine;
}

//Constructor for findsolOutput:
findsolOutput::findsolOutput()
{
    solution = Goal();
    needToChangeLine = 0;
}

findsolOutput::findsolOutput(Goal sol, bool needToChange)
{
    solution = sol;
    needToChangeLine = needToChange;
}


//------------------------------------------------------------------------
double getSteeringAngle(Position currentPosition, int currentLine, Goal goal, bool backOnLine, double speed, Goal goalsArray[], Line lines[])
{
	 ROS_INFO("Entered getsteeringAngle loop");
    double predictedAngleHistory[NUMBER_OF_PREDICTIONS];
    Goal newGoal;
    int count = 0;
    while (count<NUMBER_OF_PREDICTIONS) //predict next positions and angles
    {
		 	 ROS_INFO("%d", count);
	 		 ROS_INFO("number of predictions %d", NUMBER_OF_PREDICTIONS);
		  	 ROS_INFO("Entered while loop");
        if (currentLine == sizeof(lines)/sizeof(lines[0]))
        {
            newGoal = goalsArray[sizeof(goalsArray)/sizeof(goalsArray[0])-1];
        }
        else
        {
            updateGoalOutput output1 = updateGoal(currentPosition, currentLine, speed, backOnLine, goalsArray, lines);
            newGoal = output1.goal;
            currentLine = output1.currentLine;
        }
        double predictedAngle = calculateSteeringAngle(currentPosition, newGoal, speed);
		  ROS_INFO("Predicted angle is %f", predictedAngle);
        currentPosition = predictMovement(predictedAngle, currentPosition, speed);
        predictedAngleHistory[count] = predictedAngle;
        count ++ ;
    }
    
    double sum;
    int size;
    size = sizeof(predictedAngleHistory) / sizeof(predictedAngleHistory[0]);
	 ROS_INFO("Size: %d", size);
    sum = 0;
    double avg = 0;
    
    for(int i = 0; i < size; i++){
	 ROS_INFO("Predicted angle history [%d]: %f", i, predictedAngleHistory[i]);
        sum += predictedAngleHistory[i];
    }
    avg = sum / (double) size;
	 ROS_INFO("Average predicted angle is %f", avg);
    return avg;
};

//------------------------------------------------------------------------

double calculateSteeringAngle(Position currentPosition, Goal goal, double speed)
{
	 if (speed == 0.0) speed = 1.0;
    //the position of the car is:
    double xp = currentPosition.x;
    double yp = currentPosition.y;
    double thetap = currentPosition.theta;
    
    //the goal is:
    double xgoal = goal.x;
    double ygoal = goal.y;
    
    double theta = atan2(ygoal-yp,xgoal-xp);
    double newTheta = theta - 2*pi*floor(theta/(2*pi));
    double dtheta = (newTheta - thetap) - 2*pi*floor( (newTheta - thetap+pi)/(2*pi) );
    double calculatedAngle = atan(dtheta*car_length/(speed*TS));
    if (calculatedAngle > 6 * pi / 180) calculatedAngle = 6 * pi / 180;
    if (calculatedAngle < -6 * pi / 180) calculatedAngle = -6 * pi / 180;
    
    return calculatedAngle;
    
};

//------------------------------------------------------------------------

Position predictMovement(double steeringAngle, Position currentPosition, double speed)
{
    double TSsmall = 0.001;
    
    int number_of_iteration = TS/TSsmall;
    Position pose = currentPosition;
    
    for (int i=1; i <= number_of_iteration; i++)
    {
        double x = cos(pose.theta)*speed*TSsmall + pose.x;
        double y = sin(pose.theta)*speed*TSsmall + pose.y;
        double theta = speed/car_length*tan(steeringAngle)*TSsmall + pose.theta;
        double newTheta = theta - 2*pi*floor(theta/(2*pi));  //wrap do 2pi
        pose.x = x;
        pose.y = y;
        pose.theta = newTheta;
    }
    
    return pose;
};

//------------------------------------------------------------------------

updateGoalOutput updateGoal(Position currentPosition,  int currentLine, double speed, bool backOnLine, Goal goalsArray[], Line lines[])
{
    Line line = lines[currentLine-1]; //line i is in i-1 location inside the array
    double m = line.m;
    double q = line.q;
    double dist = DISTANCE_TO_LINE + speed/10;
    //the position of the car is:
    double xp = currentPosition.x;
    double yp = currentPosition.y;
    Goal newGoal;
    Goal solution;
    bool needtoChangeLine;
    int newCurrentLine = currentLine;
    bool newBackOnLine = backOnLine;
    
    double distance_from = std::abs(yp-(m*xp+q))/sqrt(1+m*m);//caluclate distance from line
    
    if (distance_from<dist)
    {
        findsolOutput output = findsol(currentPosition, newCurrentLine, dist, goalsArray, lines);
        solution = output.solution;
        needtoChangeLine = output.needToChangeLine;
        if (needtoChangeLine)
        {
            newCurrentLine = newCurrentLine+1; //try with next line
            line = lines[newCurrentLine-1]; //line i is in i-1 location inside the array
            m = line.m;
            q = line.q;
            
            distance_from = std::abs(yp-(m*xp+q))/sqrt(1+m*m);//caluclate distance from line
            
            if (distance_from<dist)
            {
                output = findsol(currentPosition, newCurrentLine, dist, goalsArray, lines);
                newGoal = output.solution;
            }
            else
            {
                newGoal = goalsArray[newCurrentLine];
            }
        }
        else
        {
            newGoal = solution;
            newCurrentLine = newCurrentLine;
        }
        newBackOnLine = 0;
    }
    else
    {
        newGoal = goalsArray[newCurrentLine];
        if (newBackOnLine)
        {
            newCurrentLine = newCurrentLine;
        }
        else
        {
            newCurrentLine = newCurrentLine+1;
            newBackOnLine = 1;
        }
        
    }
    updateGoalOutput outputValue(newGoal, newCurrentLine, newBackOnLine);
    return outputValue;
};


//you need to fix the fact that you have complex numbers!!!! you cannot use Position
findsolOutput findsol(Position currentPosition,int currentLine, double distance, Goal goalsArray[], Line lines[])
{
    Goal solution;
    //the car is following the i-th line:
    Line line = lines[currentLine-1];  //line i is in i-1 location inside the array
    double m = line.m;
    double q = line.q;
    
    //the position of the car is:
    double xp = currentPosition.x;
    double yp = currentPosition.y;
    
    //2nd order equation parameters
    double a = 1+m*m;
    double b = -2*xp-2*yp*m+2*m*q;
    double c = xp*xp+yp*yp+q*q-2*yp*q-distance*distance;
    
    double solx1 = (-b + sqrt(b*b-4*a*c))/(2*a);
    double solx2 = (-b - sqrt(b*b-4*a*c))/(2*a);
    double soly1 = m*solx1+q;
    double soly2 = m*solx2+q;
    
    Goal goal = goalsArray[currentLine]; //goal i is in position i inside the array
    
    double max = solx1;
    double min = solx2;
    if (solx1<solx2)
    {
        max = solx2;
        min = solx1;
    }
    
    bool needToChangeLine = goal.x<max && goal.x>min;
    
    double dist1 = sqrt((solx1-goal.x)*(solx1-goal.x)+(soly1-goal.y)*(soly1-goal.y));
    double dist2 = sqrt((solx2-goal.x)*(solx2-goal.x)+(soly2-goal.y)*(soly2-goal.y));

    
    
    //decide which of the two solutions is correct
    if (dist1<dist2)
    {
        solution.x = solx1;
        solution.y = soly1;
    }
    else
    {
        solution.x = solx2;
        solution.y = soly2;
    }
    findsolOutput output(solution, needToChangeLine);
    return output;
}

steeringAngleOutput steeringAngle(Position currentPosition, int currentLine, bool backOnLine, double speed, Goal goalsArray[], Line lines[])
{   Goal newGoal;
    bool acc;
	 int number_of_lines = sizeof(lines)/sizeof(lines[0]);
    int newCurrentLine = currentLine;
    bool newBackOnLine = backOnLine;
    if (newCurrentLine == number_of_lines)
    {   newGoal = goalsArray[number_of_lines]; }
    else
    {   updateGoalOutput output =  updateGoal(currentPosition, currentLine, speed, backOnLine, goalsArray, lines);
        newGoal = output.goal;
        newCurrentLine = output.currentLine;
        newBackOnLine = output.backOnLine;
    }
    double steeringAngle = getSteeringAngle(currentPosition,  newCurrentLine, newGoal, newBackOnLine, speed, goalsArray, lines);
	 Goal fixedGoal = goalsArray[currentLine];
    double wantedSpeed = fixedGoal.speed;
    ROS_INFO ("fixedGoal: X [%f], Y [%f], wanted speed: [%f], tol [%f]", fixedGoal.x, fixedGoal.y, fixedGoal.speed, fixedGoal.tolerance);
    double tolerance = fixedGoal.tolerance;
    float brakePower = 0;
    if(speed<wantedSpeed-tolerance)
    {
        acc = 1;
    }
	 if(speed>wantedSpeed)
    {
        acc = 0;
    }
    if (speed>wantedSpeed+tolerance)
    {
        
        brakePower = (speed-wantedSpeed-tolerance)/8.0;
    }
    steeringAngleOutput output( steeringAngle, newCurrentLine, newBackOnLine, acc, brakePower);
    std::cout <<"Angle"<< output.angle << "\n";
    return output;
	 ROS_INFO ("steeringAngle: [%f], brakePower [%f]", steeringAngle, output.brake);
};


//Constructor for steeringAngleOutput:
steeringAngleOutput::steeringAngleOutput()
{
    angle = 0.0;
    currentLine = 0;
    goingBack = 0;
    accelerate = false;
    brake = 0.0;
}

steeringAngleOutput::steeringAngleOutput(double inputAngle, int inputCurrentLine, bool inputGoingBack, bool acc, float brk)
{
    angle = inputAngle;
    currentLine = inputCurrentLine;
    goingBack = inputGoingBack;
    accelerate = acc;
    brake = brk;
}


void createLines(const Goal goalsArray[], Line lines[], int sizeofGoalsArray)
{
    
    for ( int i=1; i < sizeofGoalsArray; i++)
    {
        Goal p2 = goalsArray[i];
        Goal p1 = goalsArray[i-1];
        
        double x2 = p2.x;
        double y2 = p2.y;
        
        double x1 = p1.x;
        double y1 = p1.y;
        
        double m = (y2-y1)/(x2-x1);
        double q = -x1*m+y1;
        
        Line line(m,q);
        lines[i-1] = line;
    }
    return;
};

//------------------------------------------------------ here comes the ROS part

float messageSteeringVel = 0;
double carPath[3];
float streamTimeOdo;
float messageOdoVel;
double odoReadingTime;

void odoVelocityCallback (const dynamo_msgs::TeensyRead::ConstPtr& msg_odo_dist) {
	streamTimeOdo = 0;
	messageOdoVel = msg_odo_dist->speed_wheel;
	odoReadingTime = ros::Time::now().toSec();
}

void carPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_carPoseEstimate) {
	for (int i = 0; i < 3; i++) {
		carPath[i] = msg_carPoseEstimate->data[i];
	}
	ROS_INFO("Obtained point coordinates : [%f %f]",carPath[0], carPath[1]);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "steering_angle_calculator");

	ros::NodeHandle n;

	ros::Publisher teensyPub = n.advertise<dynamo_msgs::TeensyWrite>("teensy_write", 1);
    ros::Publisher steeringPub = n.advertise<dynamo_msgs::SteeringStepper>("cmd_steering_angle", 1);
    ros::Publisher brakePub = n.advertise<dynamo_msgs::BrakeStepper>("cmd_brake_power", 1);

	#define NR_GOALS 65

	const Goal goal0 (0,0,0,0,0);
	const Goal goal1 (1,0,0,1,0);
	const Goal goal2 (3,-0.035,0,2,0);
	const Goal goal3 (4,-0.044,0,2,0.1);
	const Goal goal4 (5,-0.2,0,2,0.1);
	const Goal goal5 (6,-0.5,0,2.5,0.1);
	const Goal goal6 (7,-0.9,0,2.5,0.1);
	const Goal goal7 (8,-1.6,0,3,0.1);
	const Goal goal8 (8.5,-2,0,3,0.1);
	const Goal goal9 (9.35,-3,0,3,0.1);
	const Goal goal10 (10,-4.3,0,3,0.1);
	const Goal goal11 (10.3,-5,0,3,0.1);
	const Goal goal12 (10.4,-6,0,3,0.1);
	const Goal goal13 (10.5,-7,0,3,0.1);
	const Goal goal14 (10.55,-9,0,3,0.1);
	const Goal goal15 (10.58,-11,0,2,0.1);
	const Goal goal16 (10.65,-15,0,2,0.1);
	const Goal goal17 (10.7,-18,0,2,0.1);
	const Goal goal18 (10.75,-21,0,2,0.1);
	const Goal goal19 (10.6,-22.45,0,3,0.1);
	const Goal goal20 (10,-24,0,3,0.1);
	const Goal goal21 (9,-25.5,0,3,0.1);
	const Goal goal22 (8.5,-26,0,3,0.1);
	const Goal goal23 (8,-26.5,0,3,0.1);
	const Goal goal24 (7.4,-27,0,3,0.1);
	const Goal goal25 (6.58,-27.5,0,3,0.1);
	const Goal goal26 (5.97,-27.8,0,2,0.1);
	const Goal goal27 (5,-28.2,0,2,0.1);
	const Goal goal28 (4,-28.4,0,2,0.1);
	const Goal goal29 (2.5,-28.4,0,2,0.1);
	const Goal goal30 (1,-28.6,0,2,0.1);
	const Goal goal31 (0,-28.6,0,2,0.1);
	const Goal goal32 (-1,-28.6,0,2,0.1);
	const Goal goal33 (-3,-28.2,0,2,0.1);
	const Goal goal34 (-4,-27.7,0,2,0.1);
	const Goal goal35 (-5,-27,0,3,0.1);
	const Goal goal36 (-5.4,-26.6,0,3,0.1);
	const Goal goal37 (-6,-25.7,0,3,0.1);
	const Goal goal38 (-6.5,-24.8,0,3,0.1);
	const Goal goal39 (-6.88,-23.5,0,3,0.1);
	const Goal goal40 (-7,-22.3,0,3,0.1);
	const Goal goal41 (-7.15,-21.2,0,3,0.1);
	const Goal goal42 (-7.2,-20.2,0,3,0.1);
	const Goal goal43 (-7.26,-19.2,0,3,0.1);
	const Goal goal44 (-7.31,-18,0,3,0.1);
	const Goal goal45 (-7.36,-16.5,0,3,0.1);
	const Goal goal46 (-7.4,-15,0,2,0.1);
	const Goal goal47 (-7.45,-13.6,0,2,0.1);
	const Goal goal48 (-7.5,-12,0,2,0.1);
	const Goal goal49 (-7.56,-11,0,2,0.1);
	const Goal goal50 (-7.6,-10,0,2,0.1);
	const Goal goal51 (-7.65,-9,0,2,0.1);
	const Goal goal52 (-7.74,-7,0,2,0.1);
	const Goal goal53 (-7.6,-5,0,3,0.1);
	const Goal goal54 (-7.3,-4,0,3,0.1);
	const Goal goal55 (-6.9,-3,0,2,0.1);
	const Goal goal56 (-6.2,-2,0,2,0.1);
	const Goal goal57 (-5.5,-1.3,0,2,0.1);
	const Goal goal58 (-4.5,-0.7,0,2,0.1);
	const Goal goal59 (-3.5,-0.3,0,2,0.1);
	const Goal goal60 (-2.5,-0.2,0,2,0.1);
	const Goal goal61 (-1.5,-0.1,0,1,0.1);
	const Goal goal62 (0.5,-0.005,0,1,0.1);
	const Goal goal63 (0.2,0,0,0.5,0.1);
	const Goal goal64 (0,0,0,0,0);

	Goal goalsArray [NR_GOALS] = {goal0, goal1, goal2, goal3, goal4, goal5,goal6,goal7,
									goal8, goal9, goal10, goal11, goal12, goal13, goal14,
			                  goal15, goal16, goal17, goal18, goal19, goal20,
									goal21, goal22, goal23, goal24, goal25, goal26, goal27, goal28,  
                           goal29, goal30, goal31, goal32, goal33, goal34, goal35, 
                           goal36, goal37, goal38, goal39, goal40, goal41, goal42,
                           goal43, goal44, goal45, goal46, goal47, goal48, goal49,
                           goal50, goal51, goal52, goal53, goal54, goal55, goal56,
                           goal57, goal58, goal59, goal60, goal61, goal62, goal63, goal64};

	// remember to fix this!!!!
	const int numberOfGoals = sizeof(goalsArray) / sizeof(goalsArray[0]);

	//Define lines out of goals:
	Line lines [numberOfGoals-1];
	createLines(goalsArray, lines, numberOfGoals);

	ros::Subscriber sub_odo_velocity = n.subscribe("teensy_read", 1, odoVelocityCallback);
	ros::Subscriber sub_carPath = n.subscribe("estimated_pose_marker", 1, carPoseCallback);
	
	float loopRate = 10; // loop rate in [Hz]
	ros::Rate r(loopRate);

	int currentLine = 1; //IMPORTANT, IT'S NOT 0!!!
	bool goingBack = 0;		


	ROS_INFO("Entering the loop");

	while (ros::ok()) {
		ros::spinOnce();
		
		double speed = messageOdoVel;

        Position currentPosition(carPath[0],carPath[1],carPath[2]);

		ROS_INFO("Obtained velocity: [%f], obtained coordinates [%f, %f, %f]",
	    messageSteeringVel, currentPosition.x, currentPosition.y,
		currentPosition.theta);

        dynamo_msgs::TeensyWrite teensyMsg;
        dynamo_msgs::SteeringStepper steeringMsg;
        dynamo_msgs::BrakeStepper brakeMsg;
		steeringAngleOutput result = steeringAngle (currentPosition, currentLine, goingBack, speed, goalsArray, lines);
		
        steeringMsg.steering_angle = result.angle * 360 / (2*M_PI);
        steeringMsg.steering_stepper_engaged = true;

		teensyMsg.burn = result.accelerate;

        brakeMsg.brake_power = result.brake;
        brakeMsg.brake_stepper_engaged = true;

		currentLine = result.currentLine;

		goingBack = result.goingBack;

		teensyPub.publish(teensyMsg);
        steeringPub.publish(steeringMsg);
        brakePub.publish(brakeMsg);

		ROS_INFO("Published steering angle: [%f], acceleration: [%d]"
 		"deceleration: [%f]", 
		steeringMsg.steering_angle, teensyMsg.burn, result.brake);

		
		r.sleep();
	}
  return 0;
}







