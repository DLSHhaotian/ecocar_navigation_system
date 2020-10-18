#include <ros/ros.h>
//#include <ros/console.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include "std_msgs/Float32MultiArray.h"
#include <dynamo_msgs/TeensyRead.h>
#include <dynamo_msgs/SteeringStepper.h>
#include <dynamo_msgs/WheelAngles.h>

#include "SteeringSystem.h"

#define DEBUG_LOGGER 1

#define CONTROL_LOOP_RATE 100

enum NodeState {
    NODE_STATE_FATAL_ERROR = -1, // For unrecoverable errors
    NODE_STATE_OK,               // When OK
    NODE_STATE_WAIT              // Node is not ready or trying to recover from an error
};

bool emergencyButtonState = false;

NodeState state;

SteeringSystem steeringSystem(CONTROL_LOOP_RATE);

// Subscriber Callbacks
void steeringAngleCallback(const dynamo_msgs::SteeringStepperPtr& msg) {
    ROS_INFO("Steering heard: [%f] %s", msg->steering_angle, msg->steering_stepper_engaged ? "engaged" : "disengaged");
    // Update the reference of the control loop
    steeringSystem.setReference(msg->steering_angle);
    steeringSystem.setEngaged(msg->steering_stepper_engaged); // && emergencyButtonState); // emergencyButton is 1 when there is no emergency!
    //if(STATE_ERROR == steeringMotorControl.updateReference(msg->steering_angle, msg->steering_stepper_engaged)) {
    //    state = NODE_STATE_FATAL_ERROR;
    //}
}


void teensyCallback(const dynamo_msgs::TeensyReadPtr& msg) {
    emergencyButtonState = msg->emergencyButton == 1;
    if(msg->emergencyButton == 0){ // emergencyButton is 1 when there is no emergency!
      //steeringSystem.setEngaged(false);
    }
    steeringSystem.updateMeasurements(msg->steering_encoder, msg->steering_potentiometer);
    steeringSystem.updateVehicleVelocity(msg->speed_wheel/3.6); // teensy data in km/h
}


/*void carPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {	 // updates global variables

  steeringSystem.updateVehicleVelocity(msg->data[4]); // velocity

}*/

bool aliveCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if(!req.data) {
        ROS_INFO("Shutdown triggered by service!");
        state = NODE_STATE_FATAL_ERROR;
    }
    res.success = (state == NODE_STATE_OK);
    return true;
}


bool calibrateSteering(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  steeringSystem.startCalibration();
  return true;
}

bool calibrateSteeringVelocity(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  steeringSystem.startVelocityCalibration();
  return true;
}

// MAIN //
int main(int argc, char *argv[]) {
  state = NODE_STATE_WAIT;
	ros::init(argc, argv, "steering_node");
	ros::NodeHandle nh;

    // Configure logger
  #if (DEBUG_LOGGER)
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  #endif

  // Prepare ROS
  ros::Subscriber steeringSub = nh.subscribe("cmd_steering_angle", 1, steeringAngleCallback);
  ros::Subscriber teensySub = nh.subscribe("teensy_read", 1, teensyCallback);
  //ros::Subscriber sub = nh.subscribe("car_pose_estimate", 1000, carPoseCallback);

  ros::Publisher wheelAnglePub = nh.advertise<dynamo_msgs::WheelAngles>("wheel_angles", 1);

  ros::ServiceServer service = nh.advertiseService("steering_alive", aliveCallback);
  ros::ServiceServer calibrationServer = nh.advertiseService("calibrate_steering", calibrateSteering);
  ros::ServiceServer velocityCalibrationServer = nh.advertiseService("calibrate_steering_velocity", calibrateSteeringVelocity);

  // ROS spin once to get initial callbacks
  ros::spinOnce();

  steeringSystem.sanityCheck();

  // Start control loop timer
  ros::Rate rate(CONTROL_LOOP_RATE);
  while(ros::ok()) {
    ros::spinOnce();

    steeringSystem.loop();

    if(state == NODE_STATE_FATAL_ERROR) {
      ROS_ERROR("Steering node going down");
      break;
    }
    state = NODE_STATE_OK;

    // Publish "measured" angles
    dynamo_msgs::WheelAngles msg;
    msg.header.stamp = ros::Time::now();
    msg.leftAngle = steeringSystem.getLeftAngle();
    msg.rightAngle = steeringSystem.getRightAngle();
    msg.steering_velocity = steeringSystem.getSteeringSpeed();
    msg.is_skipping = steeringSystem.isSkipping();
    msg.is_slipping = steeringSystem.isSlipping();
    wheelAnglePub.publish(msg);

    rate.sleep();
  }

	// Close down connection
  steeringSystem.close();
  return 0;
}
