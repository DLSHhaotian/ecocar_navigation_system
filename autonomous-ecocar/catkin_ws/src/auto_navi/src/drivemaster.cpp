#include "ros/ros.h"
//#include "autonomous-ecocar/catkin_ws/src/navigation/global_variables.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "lidar_package/point.h"
#include "lidar_package/obst.h"
#include "lidar_package/obsts.h"
#include <cstdio>
#include <vector>
#include <iostream>
#include <fstream>
#include <ctime>
#include "beginner_tutorials/commandMsg.h"
#include "beginner_tutorials/flagMsg.h"
//using namespace std;
 
// Author: Jesper C, s130580, 2018-03-22
// Description: this node takes the car odometry messages and saves them in a log-file (.txt)
 
ros::Publisher master_pub;
 
double drivendist, theta, current_flag;
int ct, noco;
int i;
double td, scm;
int flag, no_of_commands;
std::vector<int> comtyp;
std::vector<double> tardis;   
 


 
void missionScript() {
	
    comtyp.push_back(1);
    tardis.push_back(10.0);
     
    comtyp.push_back(2);
    tardis.push_back(10.0);
     
    comtyp.push_back(3);
    tardis.push_back(10.0);
    
	
    comtyp.push_back(99);
    tardis.push_back(0.0);
     
     
}
 
 



void commandSender() {
    beginner_tutorials::commandMsg comand;
     
    if (flag == comtyp[i]) {
        // go to next command
        std::cout<<"\n\n\n* * *  NEXT COMMAND  * * *\n";
        // maybe delay or sleep here
        i++;
        if (flag == 99) std::cout <<"\n\n * * * scanmaker terminated * * * \n\n"; // probably never comes true since scanmaker shuts down before sending
    }
    ct = comtyp[i];
    td = tardis[i];
     
    if (noco != i) {
        noco = i;
        std::cout << "\nSent new command:  no. "<< i  << "  ct: " << ct << "   td:   " << td << "\n";
    }
     
    comand.comtype = ct;
    comand.targetdist = td;
    master_pub.publish(comand);
     
}
 
 


void flagCallback(const beginner_tutorials::flagMsg::ConstPtr& msg) {
    flag = msg->flagtype;        // receiving flag X means that scanmaker has completed the state with command_type X.
    scm = msg->command_number;   // no. of previously completed commands
     
    if (flag != current_flag) {
        current_flag = flag;
        std::cout << "\nReceived new flag: " << flag << "  with command_number: " << scm << "\n";
    }
     
    commandSender();
}
 




void carPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_carPoseEstimate) {	 // updates global variables
     
    // carVal[0]: x
    // carVal[1]: y
    // carVal[3]: theta [rad] 
    // carVal[6]: drivendist
    drivendist = msg_carPoseEstimate->data[6];   // updates global variable
    theta = msg_carPoseEstimate->data[3];    
	
}
 
 
 


int main(int argc, char **argv) {
    drivendist = 0.0;   // global variable for current drivendist
    theta = 0.0;        // global variable for current car orientation (odotheta)
    ct = 0;         // global variable for current command type
    td = 0.0;       // global variable for targetdist
    i = 0;      // global variable for current index in command vector
    scm = 0;    //scanmaker command number
    flag = 0;
    current_flag = 0;
    noco = 77;
     
    // maybe do all this in a function instead of main()
     
    missionScript();    // load commands into global command vectors
    no_of_commands = comtyp.size(); // the total number of commands in missionScript()      // unsuesd right now
     
 
    ros::init(argc, argv, "drivemaster");
    ros::NodeHandle n;
 
    ros::Subscriber sub = n.subscribe("car_pose_estimate", 1000, carPoseCallback);   // modtag messages på topic "car_pose_estimate" og send til funktionen carPoseCallback
    ros::Subscriber sub2 = n.subscribe("scanmaker_flags", 10, flagCallback);
    master_pub = n.advertise<beginner_tutorials::commandMsg>("master_commands",5);    // publish msgs of type "driveMsg" to topic "dripve_points"
 
    ros::spin();
 
    return 0;
}
