#include "EcocarPlugin.h"
#include <queue>

using namespace gazebo;

EcocarPlugin::EcocarPlugin()
{
  if (!ros::isInitialized())
  {
    int argc = 0;
    char* argv = nullptr;
    ros::init(argc, &argv, "EcocarGazeboPlugin");
  }
  ROS_DEBUG("Init EcocarPlugin");
  
  teensySub = nh.subscribe("teensy_write", 10, &EcocarPlugin::onTeensyCommand, this);
  brakeSub = nh.subscribe("cmd_brake_power", 10, &EcocarPlugin::onBrakeCommand, this);
  steeringSub = nh.subscribe("cmd_steering_angle", 10, &EcocarPlugin::onSteeringCommand, this);

  steeringPIDSub = nh.subscribe("sim/setSteeringPID", 10, &EcocarPlugin::onSetSteeringPIDCommand, this);

  wheelPub = nh.advertise<ecocar_gazebo_msgs::WheelData>("sim/raw/distance_wheel",1);

  posePub = nh.advertise<geometry_msgs::Pose>("sim/debug/pose", 1000);
  motorPub = nh.advertise<ecocar_gazebo_msgs::MotorData>("sim/debug/motor", 1000);

  steeringPub = nh.advertise<ecocar_gazebo_msgs::SteeringData>("sim/debug/steering", 1000);

  #if ENABLE_DRAG_FORCE
  dragPub = nh.advertise<ecocar_gazebo_msgs::DragData>("sim/debug/drag", 1000);
  #endif

  #if ENABLE_ROLLING_RESISTANCE
  rrPub = nh.advertise<ecocar_gazebo_msgs::RollingResistanceData>("sim/debug/rolling_resistance", 1000);
  #endif

  lastDebugMsgTime = 0;
  lastSensorMsgTime = 0;
}

void EcocarPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_DEBUG("EcocarPlugin::Load()");
  this->model = _model;
  this->world = model->GetWorld();

  // Set a function to run at world update, EcocarPlugin::onUpdate()
  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&EcocarPlugin::onUpdate, this));
  resetConnection = event::Events::ConnectWorldReset(std::bind(&EcocarPlugin::onReset, this));

  // Retrieve links and joints
  // Chassis link pointer
  std::string chassisLinkName = model->GetName() + "::" + "base_link";
  chassisLink = model->GetLink(chassisLinkName);
  if(!chassisLink){
    ROS_ERROR("Could not find chassis link");
  }

  // Find joints which are to be controlled with the plugin
  // Front left wheel
  std::string flWheelJointName = model->GetName() + "::" + "front_left_wheel_joint";
  flWheelJoint = model->GetJoint(flWheelJointName);
  if(!flWheelJoint){
    ROS_ERROR("Could not find front left wheel joint");
  }

  // Front right wheel
  std::string frWheelJointName = model->GetName() + "::" + "front_right_wheel_joint";
  frWheelJoint = model->GetJoint(frWheelJointName);
  if(!frWheelJoint){
    ROS_ERROR("Could not find front right wheel joint");
  }

  // Front left steering
  std::string flSteeringJointName = model->GetName() + "::" + "front_left_steer_joint";
  flSteeringJoint = model->GetJoint(flSteeringJointName);
  if(!flSteeringJoint){
    ROS_ERROR("Could not find front left steering joint");
  }

  // Front right steering
  std::string frSteeringJointName = model->GetName() + "::" + "front_right_steer_joint";
  frSteeringJoint = model->GetJoint(frSteeringJointName);
  if(!frSteeringJoint){
    ROS_ERROR("Could not find front right steering joint");
  }

  // Rear left wheel
  std::string blWheelJointName = model->GetName() + "::" + "susp_to_wheel_rear_left";
  blWheelJoint = model->GetJoint(blWheelJointName);
  if(!blWheelJoint){
    std::string blWheelJointName = model->GetName() + "::" + "base_to_wheel_rear_left";
    blWheelJoint = model->GetJoint(blWheelJointName);
    if(!blWheelJoint){
      ROS_ERROR("Could not find rear left wheel joint");
    }
  }

  // Rear right wheel
  std::string brWheelJointName = model->GetName() + "::" + "susp_to_wheel_rear_right";
  brWheelJoint = model->GetJoint(brWheelJointName);
  if(!brWheelJoint){
    std::string brWheelJointName = model->GetName() + "::" + "base_to_wheel_rear_right";
    brWheelJoint = model->GetJoint(brWheelJointName);
    if(!brWheelJoint){
      ROS_ERROR("Could not find rear right wheel joint");
    }
  }

  // Retrieve parameters
  //this->frontTrackWidth = 1.084;
  unsigned int id = 0;
  this->frontTrackWidth = 2*flSteeringJoint->Anchor(id).Y();
  ROS_DEBUG("Front track width: %f", this->frontTrackWidth);

  //this->wheelBaseLength = 1.516;
  this->wheelBaseLength = 2*flSteeringJoint->Anchor(id).X();
  ROS_DEBUG("Wheel base length: %f", this->wheelBaseLength);

  //this->wheelRadius = 0.2794;
  physics::CylinderShape *cyl = static_cast<physics::CylinderShape*>(flWheelJoint->GetChild()->GetCollision(id)->GetShape().get());
  this->wheelRadius = cyl->GetRadius();
  ROS_DEBUG("Wheel radius: %f", this->wheelRadius);

  // Derived values
  this->wheelCircumference = 2*IGN_PI*this->wheelRadius;

  //
  this->steerMin = -45.0;
  this->steerMax = 45.0;

  // Implement those as settings in the xacro sfile..
  std::string paramName;
  //const double Kp = 100;
  double Kp, Ki, Kd, kMaxSteeringForceMagnitude;

  paramName = "steering_p_gain";
  if(_sdf->HasElement(paramName))
    Kp = _sdf->Get<double>(paramName);
  else
    Kp = 0;

  paramName = "steering_i_gain";
  if(_sdf->HasElement(paramName))
    Ki = _sdf->Get<double>(paramName);
  else
    Ki = 0;

  paramName = "steering_d_gain";
  if(_sdf->HasElement(paramName))
    Kd = _sdf->Get<double>(paramName);
  else
    Kd = 0;

  ROS_DEBUG("Steering P,I,D: %f, %f, %f", Kp, Ki, Kd);


  paramName = "steering_max_force";
  if(_sdf->HasElement(paramName))
    kMaxSteeringForceMagnitude = _sdf->Get<double>(paramName);
  else
    kMaxSteeringForceMagnitude = 0;

  paramName = "max_brake_pressure";
  if(_sdf->HasElement(paramName))
    max_brake_pressure = _sdf->Get<double>(paramName);
  else
    max_brake_pressure = 0;

  paramName = "rho_air";
  if(_sdf->HasElement(paramName))
    rho_air = _sdf->Get<double>(paramName);
  else
    rho_air = 0;

  paramName = "gravity";
  if(_sdf->HasElement(paramName))
    g = _sdf->Get<double>(paramName);
  else
    g = 9.8;

  // Set PID controllers for steering joints
  flWheelSteeringPID.SetPGain(Kp);
  flWheelSteeringPID.SetIGain(Ki);
  flWheelSteeringPID.SetDGain(Kd);
  flWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  flWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  frWheelSteeringPID.SetPGain(Kp);
  frWheelSteeringPID.SetIGain(Ki);
  frWheelSteeringPID.SetDGain(Kd);
  frWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  frWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  // Get inertia from model
  const double I_wheel = blWheelJoint->GetChild()->GetInertial()->IZZ();

  #if ENABLE_MOTOR_SIMULATION
  // Should be the same for both wheels..
  this->motor = new Motor(I_wheel);
  ROS_DEBUG("I_wheel: %f",I_wheel);
  #endif

  // Find total mass (with dummy masses of joints etc.)
  // Childs of "base_link"
  physics::Link_V links = chassisLink->GetChildJointsLinks();
  std::queue<physics::LinkPtr> linkqueue;

  for(auto e : links){
    linkqueue.push(e);
  }

  double temp = 0;
  // Traverse through all childs
  while(!linkqueue.empty()){
    ROS_DEBUG("Found link: %s", linkqueue.front()->GetName().c_str());
    ROS_DEBUG("Mass: %f", linkqueue.front()->GetInertial()->Mass());
    temp += linkqueue.front()->GetInertial()->Mass();

    // Find child links of this one and add to queue
    physics::Link_V newlinks = linkqueue.front()->GetChildJointsLinks();
    for(auto e : newlinks){
      linkqueue.push(e);
    }

    linkqueue.pop();
  }

  totalMass = temp + chassisLink->GetInertial()->Mass();

  Itot = 4*I_wheel + totalMass * pow(this->wheelRadius,2);

  ROS_DEBUG("Total mass: %f", totalMass);

  // Initial values
  // Wheel angle
  this->wheelAngle = 0;
  // Wheel encoder counter
  this->wheelCount = 0;

  this->burn = false;
  this->steeringAngle = 0;
  this->brakingPower = 0;
  calculateSteeringAngles();

  ROS_INFO("Finished loading Ecocar Plugin.");

}

void EcocarPlugin::onTeensyCommand(const dynamo_msgs::TeensyWritePtr& msg)
{
  /*
  ROS_DEBUG("EcocarPlugin recieved:");
  ROS_DEBUG("Burn: %s", msg->burn?"true":"false");
  */

  // Set internal parameters to be used in onUpdate() function
  this->burn = msg->burn;
}

void EcocarPlugin::onSteeringCommand(const dynamo_msgs::SteeringStepperPtr& msg)
{
  /*
  ROS_DEBUG("EcocarPlugin recieved:");
  ROS_DEBUG("Steering stepper engaged: %s", msg->steering_stepper_engaged?"true":"false");
  ROS_DEBUG("Steering angle: %f", msg->steering_angle);
  */

  // Set internal parameters to be used in onUpdate() function
  if(msg->steering_stepper_engaged){
    this->steeringAngle = ignition::math::clamp((double)msg->steering_angle, this->steerMin, this->steerMax); // clamp and convert to radians
    calculateSteeringAngles();
  }   // disengaged could be used to set steering joint torques to zero
}

void EcocarPlugin::onBrakeCommand(const dynamo_msgs::BrakeStepperPtr& msg)
{
  /*
  ROS_DEBUG("EcocarPlugin recieved:");
  ROS_DEBUG("Brake stepper engaged: %s", msg->brake_stepper_engaged?"true":"false");
  ROS_DEBUG("Braking power: %f", msg->brake_power);
  */

  // Set internal parameters to be used in onUpdate() function
  if(msg->brake_stepper_engaged){
    this->brakingPower = ignition::math::clamp(msg->brake_power, 0.0f, (float)this->max_brake_pressure);
  }
}

void EcocarPlugin::onSetSteeringPIDCommand(const ecocar_gazebo_msgs::SteeringPIDPtr& msg){
  flWheelSteeringPID.SetPGain(msg->Kp);
  flWheelSteeringPID.SetIGain(msg->Ki);
  flWheelSteeringPID.SetDGain(msg->Kd);

  frWheelSteeringPID.SetPGain(msg->Kp);
  frWheelSteeringPID.SetIGain(msg->Ki);
  frWheelSteeringPID.SetDGain(msg->Kd);
}

void EcocarPlugin::calculateSteeringAngles()
{
  // Calculate Ackermann steering angles from steering wheel angle
  const double tanSteer = tan(this->steeringAngle*IGN_PI/180.0);

  this->lSteeringAngle = atan2(tanSteer,1-frontTrackWidth/(2*wheelBaseLength)*tanSteer);
  //ROS_DEBUG("Left steering angle: %f", this->lSteeringAngle);

  this->rSteeringAngle = atan2(tanSteer,1+frontTrackWidth/(2*wheelBaseLength)*tanSteer);
  //ROS_DEBUG("Right steering angle: %f", this->rSteeringAngle);
}

void EcocarPlugin::onUpdate()
{
  // This function is called every timestep of the simulation
  // Calculate the step time
  const common::Time curTime = this->world->SimTime();
  const double dt = (curTime - this->lastSimTime).Double();

  ros::Time msgTime(curTime.Double());

  // Save last time
  this->lastSimTime = curTime;

  ignition::math::Vector3d linearVel = this->model->WorldLinearVel();
  ignition::math::Vector3d relativeVel = this->model->RelativeLinearVel();

  ignition::math::Vector3d linearVelNormalized = this->model->WorldLinearVel().Normalize();
  ignition::math::Vector3d relativeVelNormalized = this->model->RelativeLinearVel().Normalize();

  //const double linearSpeed = linearVel.GetLength();

  // Get wheel velocities
  const double omega_l = this->blWheelJoint->GetVelocity(0);
  const double omega_r = this->brWheelJoint->GetVelocity(0);

  const double omega_avg = (omega_l+omega_r)/2.0;

  // Discretized odometry counts
  this->wheelAngle += omega_r*dt;
  this->wheelCount = wheelAngle/(2.0*IGN_PI) * WHEEL_COUNTS_PER_REV; // integer
  //ROS_DEBUG("Wheel count: %li", wheelcount);

  // Burn logic
  double wheelTorque;
  if(this->burn){
    #if ENABLE_MOTOR_SIMULATION
    // Motor torque simulation
    // Torque on each wheel
    wheelTorque = motor->getTorque(omega_avg);

    #else
    // Dummy constant torque
    wheelTorque = CONSTANT_TORQUE;
    #endif
  } else {
    wheelTorque = 0;
  }

  // Only on rear right wheel
  brWheelJoint->SetForce(0, wheelTorque);

  // Publish motor data
  ecocar_gazebo_msgs::MotorData motormsg;
  motormsg.header.stamp = msgTime;
  motormsg.omega_wheel = omega_avg;
  motormsg.wheel_torque = wheelTorque;
  motorPub.publish(motormsg);

  // TODO rate limited steering
  // Steering PID control
  const double lCurSteeringAngle = flSteeringJoint->Position(0);
  const double rCurSteeringAngle = frSteeringJoint->Position(0);

  const double lwsError = lCurSteeringAngle - this->lSteeringAngle;
  const double lwsCmd = this->flWheelSteeringPID.Update(lwsError, dt);
  flSteeringJoint->SetForce(0, lwsCmd);

  const double rwsError = rCurSteeringAngle - this->rSteeringAngle;
  const double rwsCmd = this->frWheelSteeringPID.Update(rwsError, dt);
  frSteeringJoint->SetForce(0, rwsCmd);

  // debug info for pid controller
  ecocar_gazebo_msgs::SteeringData steeringmsg;
  steeringmsg.header.stamp = msgTime;
  steeringmsg.current_angle_left = ignition::math::Angle(flSteeringJoint->Position(0)).Degree();
  steeringmsg.current_angle_right = ignition::math::Angle(frSteeringJoint->Position(0)).Degree();

  steeringmsg.command_angle= this->steeringAngle;

  steeringmsg.torque_left = lwsCmd;
  steeringmsg.torque_right = rwsCmd;
  steeringPub.publish(steeringmsg);

  // Braking
  // Braking power in bar
  double deacc = 1.0/3.6 * this->brakingPower;
  double brake_friction = deacc/this->wheelRadius * this->Itot/4.0;

  this->flWheelJoint->SetParam("friction", 0, brake_friction);
  this->frWheelJoint->SetParam("friction", 0, brake_friction);
  this->blWheelJoint->SetParam("friction", 0, brake_friction);
  this->brWheelJoint->SetParam("friction", 0, brake_friction);


  #if ENABLE_DRAG_FORCE
  // Drag force - data from wind tunnel
  // Drag coefficient and front area depends on angle of attack

  // This is the angle between the heading and the velocity (in radians)
  double phi = atan2(relativeVel.Y(), relativeVel.X());

  double A_front = drag.getA(phi);
  double Cd = drag.getCd(phi);
  //const double A_front = 0.8575;
  //const double Cd = 0.1874;
  math::Vector3 dragForce = -0.5 * rho_air * A_front * Cd * linearVel.GetSquaredLength() * linearVelNormalized;
  this->chassisLink->AddForce(dragForce); // could we use setparam friction instead.

  ecocar_gazebo_msgs::DragData dragMsg;
  dragMsg.header.stamp = msgTime;
  dragMsg.phi = phi;
  dragMsg.A_front = A_front;
  dragMsg.Cd = Cd;
  dragMsg.drag_force = dragForce.GetLength();
  dragPub.publish(dragMsg);

  #endif


  #if ENABLE_ROLLING_RESISTANCE
  // Rolling resistance
  // How do I stop this from making the car moving in random directions when it is standing still? This effect is quite small
  // could we use setparam friction instead?

  // Get radial acceleration
  math::Vector3 acc = this->model->GetRelativeLinearAccel();
  math::Vector3 tangentAcc = acc.Dot(relativeVelNormalized) * relativeVelNormalized;
  math::Vector3 radialAcc = acc - tangentAcc;
  double beta =  0.0425*radialAcc.GetLength();
  double Crr = 0.001*beta*beta + 0.002*beta + 0.0015;

  math::Vector3 rollingResistance = -Crr * this->totalMass * g * linearVelNormalized;
  this->chassisLink->AddForce(rollingResistance);

  ecocar_gazebo_msgs::RollingResistanceData rrMsg;
  rrMsg.header.stamp = msgTime;
  rrMsg.a_r = radialAcc.GetLength();
  rrMsg.beta = beta;
  rrMsg.Crr = Crr;
  rrMsg.rolling_resistance = rollingResistance.GetLength();
  rrPub.publish(rrMsg);

  #endif

  // Publish sensor data
  if(curTime - this->lastSensorMsgTime >= 1.0/ODO_PUB_FREQ){
    // Odometry
    ecocar_gazebo_msgs::WheelData wheelMsg;

    wheelMsg.header.stamp = msgTime;
    wheelMsg.distance_wheel = (this->wheelCount * this->wheelCircumference) / WHEEL_COUNTS_PER_REV;
    wheelPub.publish(wheelMsg);

    this->lastSensorMsgTime += 1.0/ODO_PUB_FREQ;
  }

  // Debug pose publishing
  if(curTime - this->lastDebugMsgTime > 1.0/DEBUG_PUB_FREQ){
    geometry_msgs::Pose pose;
    geometry_msgs::Point pos;
    geometry_msgs::Quaternion quat;
    pos.x = this->model->WorldPose().Pos().X();
    pos.y = this->model->WorldPose().Pos().Y();
    pos.z = this->model->WorldPose().Pos().Z();
    quat.x = this->model->WorldPose().Rot().X();
    quat.y = this->model->WorldPose().Rot().Y();
    quat.z = this->model->WorldPose().Rot().Z();
    quat.w = this->model->WorldPose().Rot().W();
    pose.position = pos;
    pose.orientation = quat;
    posePub.publish(pose);

    this->lastDebugMsgTime = curTime;
  }

}

void EcocarPlugin::onReset(){
  this->lastDebugMsgTime = 0;
  this->lastSensorMsgTime = 0;
}

GZ_REGISTER_MODEL_PLUGIN(EcocarPlugin)
