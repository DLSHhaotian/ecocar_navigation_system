/*
    Gazebo plugin & ros node for the simulated autonomous ecocar
    TODO:
      - Steering rate limit

    Author: Thomas Passer Jensen (s134234@student.dtu.dk)
*/

#include <gazebo/gazebo.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/PID.hh>

#include <ros/ros.h>
#include "dynamo_msgs/BrakeStepper.h"
#include "dynamo_msgs/SteeringStepper.h"
#include "dynamo_msgs/TeensyWrite.h"
#include "dynamo_msgs/TeensyRead.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"

#include "ecocar_gazebo_msgs/WheelData.h"
#include "ecocar_gazebo_msgs/MotorData.h"
#include "ecocar_gazebo_msgs/DragData.h"
#include "ecocar_gazebo_msgs/RollingResistanceData.h"
#include "ecocar_gazebo_msgs/SteeringData.h"
#include "ecocar_gazebo_msgs/SteeringPID.h"

#include "Motor.h"
#include "Drag.h"

namespace gazebo

// Time between publishing
#define ODO_PUB_FREQ 1000.0
#define DEBUG_PUB_FREQ 100.0

// Enable different elements of the simulation for testing purposes
#define ENABLE_DRAG_FORCE 0
#define ENABLE_ROLLING_RESISTANCE 0
#define ENABLE_MOTOR_SIMULATION 1

// If motor simulation is disabled, define a constant wheel torque
#define CONSTANT_TORQUE 50;

#define WHEEL_COUNTS_PER_REV 60.0

{
  class EcocarPlugin : public ModelPlugin
  {
    public:
      EcocarPlugin();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
      void onTeensyCommand(const dynamo_msgs::TeensyWritePtr& msg);
      void onBrakeCommand(const dynamo_msgs::BrakeStepperPtr& msg);
      void onSteeringCommand(const dynamo_msgs::SteeringStepperPtr& msg);
      void onSetSteeringPIDCommand(const ecocar_gazebo_msgs::SteeringPIDPtr& msg);
      void onUpdate();
      void onReset();

    private:
      ros::NodeHandle nh;
      ros::Subscriber teensySub;
      ros::Subscriber brakeSub;
      ros::Subscriber steeringSub;
      ros::Subscriber steeringPIDSub;

      ros::Publisher posePub;
      ros::Publisher motorPub;
      ros::Publisher wheelPub;
      ros::Publisher steeringPub;

      event::ConnectionPtr updateConnection;
      event::ConnectionPtr resetConnection;

      physics::WorldPtr world;
      physics::ModelPtr model;
      physics::LinkPtr chassisLink;
      physics::JointPtr blWheelJoint;
      physics::JointPtr brWheelJoint;
      physics::JointPtr flWheelJoint;
      physics::JointPtr frWheelJoint;
      physics::JointPtr flSteeringJoint;
      physics::JointPtr frSteeringJoint;

      common::PID flWheelSteeringPID;
      common::PID frWheelSteeringPID;
      common::Time lastSimTime;
      common::Time lastSensorMsgTime;
      common::Time lastDebugMsgTime;

      //sensors::NoisePtr gyroNoise;

      bool burn;
      double steeringAngle;
      double brakingPower;
      double lSteeringAngle;
      double rSteeringAngle;

      double g;
      double rho_air;
      double max_brake_pressure;
      double steerMin;
      double steerMax;
      double frontTrackWidth;
      double wheelBaseLength;
      double wheelRadius;
      double wheelCircumference;
      double wheelAngle;
      long int wheelCount;
      double totalMass;
      double Itot;

      void calculateSteeringAngles();

      #if ENABLE_MOTOR_SIMULATION
      Motor* motor;
      #endif

      #if ENABLE_DRAG_FORCE
      Drag drag;
      ros::Publisher dragPub;
      #endif

      #if ENABLE_ROLLING_RESISTANCE
      ros::Publisher rrPub;
      #endif
  };
}
