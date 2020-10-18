#include "SteeringSystem.h"

#include <iostream>

//begin Steering Event Handlers //
int CCONV SteeringAttachHandler(CPhidgetHandle stepper, void *userptr) {
    ROS_INFO("Steering: Stepper attached!\n");
    return 0;
}
int CCONV SteeringDetachHandler(CPhidgetHandle stepper, void *userptr) {
    ROS_WARN("Steering: Stepper detached!\n");
    // TODO Handle error
    return 0;
}
int CCONV SteeringErrorHandler(CPhidgetHandle stepper, void *userptr, int ErrorCode, const char *Description) {
    ROS_ERROR("Steering: Error handled. %d - %s\n", ErrorCode, Description);
    // TODO Handle error
    return 0;
}
//end Steering Event Handlers //

SteeringSystem::SteeringSystem(double frequency) :
  potLowpass(LOW_PASS_TIME_CONSTANT, 1.0/frequency),
  r1(15, 1.0/frequency),
  r2(15, 1.0/frequency),
  g1(0,CALIBRATION_VELOCITY*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION,0.01,500), // parameters to be determined for g1
  g2(0,CALIBRATION_VELOCITY*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION*PINION_RADIUS,1.0,350)
  {
  controllerState = STATE_INIT; // stay in init until first message from teensy is recieved

  engaged = false;

  vehicleVelocity = 0;

  potZero = 0;
  hasCalibration = loadConfig();

  diagInit = false;
  moving = false;

  potValue = 0;
  angleReference = 0;
  thetaZero = calcRightAngle(0);

  if(hasCalibration){
    ROS_INFO("Found steering system configuration file.");
    potReference = displacementToPot(calcDisplacement(angleReference));
    g1.setParams(0, TARGET_VELOCITY*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION, 35,50);
    g2.setParams(0, TARGET_VELOCITY*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION*PINION_RADIUS, 4.7, 100);
  } else {
    ROS_INFO("No steering configuration file found.");
    stepperVelocity = TARGET_VELOCITY;
  }

  #if LOGGING
  logfile.open("steeringdata.log");
  #endif

  initStepper();

  // Calculations for stepper velocity to angle conversion
  double c1 = (RACK_LENGTH-WIDTH)*(RACK_LENGTH-WIDTH) + 4*OFFSET*OFFSET;
  double c2 = 4*(TIE_ROD_LENGTH*TIE_ROD_LENGTH-STEERING_ARM_LENGTH*STEERING_ARM_LENGTH);
  //ROS_INFO("c1: %f", c1);
  //ROS_INFO("c2: %f", c2);

  double slope = 2.0/c1 * (2*OFFSET - ((RACK_LENGTH-WIDTH)*(c1+c2))/sqrt(16*STEERING_ARM_LENGTH*STEERING_ARM_LENGTH*c1-(c1-c2)*(c1-c2)) ); // rad/mm

  //ROS_INFO("Slope: %f", slope);
  stepsToAngleSpeedCoefficient = slope * 2*180.0*PINION_RADIUS/STEPPER_STEPS_PER_REVOLUTION; // deg/step
  ROS_INFO("Steps to angle speed coefficient: %f", stepsToAngleSpeedCoefficient);

}

void SteeringSystem::initStepper() {
    //state = STATE_INIT;
    CPhidgetStepper_create(&steeringStepper);
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)steeringStepper, SteeringAttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)steeringStepper, SteeringDetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)steeringStepper, SteeringErrorHandler, NULL);
    CPhidget_open((CPhidgetHandle)steeringStepper, STEERING_PHIDGET_SERIAL);
    ROS_INFO("Waiting for Steering Phidget to be attached....");

    int result;
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)steeringStepper, WAIT_FOR_ATTACH_TIMEOUT))) {
        const char *err;
        CPhidget_getErrorDescription(result, &err);
        ROS_ERROR("Problem waiting for Steering attachment: %s\n", err);
    }
    CPhidgetStepper_setCurrentLimit(steeringStepper, 0, MAX_CURRENT);
    // Default values
    CPhidgetStepper_setCurrentPosition(steeringStepper, 0, 0);
    ROS_INFO("Steering stepper initialization done.");
}

bool SteeringSystem::loadConfig(){
  bool foundConfig = true;

  if (boost::filesystem::exists("steering_system.ini")){
    using boost::property_tree::ptree;
    ptree pt;
    read_ini("steering_system.ini", pt);

    if(pt.get_optional<boost::posix_time::ptime>("timestamp").is_initialized()){
      boost::posix_time::ptime calibrationtime = pt.get<boost::posix_time::ptime>("timestamp");
      ROS_INFO("Loading config file from %s ...", to_simple_string(calibrationtime).c_str());
    }

    if(pt.get_optional<double>("potzero").is_initialized()){
      potZero = pt.get<double>("potzero");
      ROS_INFO("potzero=%f", potZero);
    } else {
      foundConfig = false;
    }

    if(pt.get_optional<double>("potmin").is_initialized()){
      potMin = pt.get<double>("potmin");
      ROS_INFO("potmin=%f", potMin);
    } else {
      foundConfig = false;
    }

    if(pt.get_optional<double>("potmax").is_initialized()){
      potMax = pt.get<double>("potmax");
      ROS_INFO("potmax=%f", potMax);
    } else {
      foundConfig = false;
    }
    if(pt.get_optional<unsigned int>("vel").is_initialized()){
      stepperVelocity = pt.get<unsigned int>("vel");
      ROS_INFO("vel=%u", stepperVelocity);
    } else {
      stepperVelocity = TARGET_VELOCITY; // use constant if none saved.
      //foundConfig = false;
    }

  } else {
    foundConfig = false;
  }
  return foundConfig;
}

void SteeringSystem::saveConfig(){
  using boost::property_tree::ptree;

  ptree pt;
  pt.put("potzero", potZero);
  pt.put("potmin", potMin);
  pt.put("potmax", potMax);
  pt.put("vel", stepperVelocity);
  pt.put("timestamp", ros::Time::now().toBoost());

  write_ini("steering_system.ini", pt );
  ROS_INFO("Saved steering system configuration file.");
}

void SteeringSystem::setReference(double angle){
  if(hasCalibration){
    this->angleReference = angle;
      ros::Time before = ros::Time::now();
      potReference = displacementToPot(calcDisplacement(angle));
      if(potReference > potMax){
        potReference = potMax;
        ROS_WARN("Angle was truncated to: %f, %f", calcLeftAngle(potToDisplacement(potReference)), calcRightAngle(potToDisplacement(potReference)));
      }
      if(potReference < potMin){
        potReference = potMin;
        ROS_WARN("Angle was truncated to: %f, %f", calcLeftAngle(potToDisplacement(potReference)), calcRightAngle(potToDisplacement(potReference)));
      }

      ROS_INFO("Updated reference, virtual angle: %f, pot reference: %f", angle, potReference);
  } else {
    ROS_WARN("Cannot command steering angle when the steering system is not calibrated.");
  }

}

void SteeringSystem::setEngaged(bool engaged){
  ROS_INFO("Setting engaged from recieved msg: %s", engaged ? "true" : "false");
  this->engaged = engaged;
}

void SteeringSystem::updateMeasurements(double encoderCount, double potValue){

  if(controllerState == STATE_INIT){
    ROS_INFO("Recieved first measurement.");
    // First measurement
    countZero = encoderCount;
    
    potLowpass.set(potValue);

    if(hasCalibration){
      controllerState = STATE_OK;
    } else {
      potZero = potValue;
      controllerState = STATE_MISSING_CALIBRATION;
    }
  }

  this->encoderCount = encoderCount;

  if(checkPotentiometerValue(potValue)){
    if(controllerState == STATE_NO_FEEDBACK){
      ROS_INFO("Teensy connection OK again.");
      diagInit = false; // reinit diag system

      if(hasCalibration){
        controllerState = STATE_OK;
      } else {
        controllerState = STATE_MISSING_CALIBRATION;
      }
    }

    this->potValue = potValue;
  } else {
    ROS_WARN_THROTTLE(1,"Got potentiometer value out of bounds - is it connected?");
    controllerState = STATE_NO_FEEDBACK;
  }

  lastTeensyTime = ros::Time::now();
}

void SteeringSystem::updateVehicleVelocity(double vel){
  vehicleVelocity = vel;
}


void SteeringSystem::startCalibration(){
  if(controllerState == STATE_OK || controllerState == STATE_MISSING_CALIBRATION){
    ROS_INFO("Starting calibration...");
    controllerState = STATE_CALIBRATING_LIMITS;
    calibrationState = GOING_LEFT;
    g1.setParams(0,CALIBRATION_VELOCITY*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION, 0.01,500);
    g2.setParams(0,CALIBRATION_VELOCITY*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION*PINION_RADIUS,1.0,350);
  } else {
    ROS_WARN("Cannot calibrate in this controller state!");
  }
}

void SteeringSystem::startVelocityCalibration(){
  if(controllerState == STATE_OK){
    stepperVelocity = TARGET_VELOCITY;
    g1.setParams(0,stepperVelocity*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION, 0.01,500);
    g2.setParams(0,stepperVelocity*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION*PINION_RADIUS,1.0,350);
    // need limit calibration first
    ROS_INFO("Starting velocity calibration...");
    timesWithoutSlip = 0;
    controllerState = STATE_CALIBRATING_VELOCITY;
    calibrationState = GOING_LEFT;
  } else {
    ROS_WARN("Cannot calibrate steering velocity before limits are calibrated (or while having other problems)!");
  }
}

void SteeringSystem::loop(){
  potLowpass.update(potValue); // TODO Use this value in the rest of the code
  if(CPhidgetStepper_getCurrentPosition(steeringStepper, 0, &currentStepPosition) == EPHIDGET_OK) {
        ROS_DEBUG_THROTTLE(1,"Steering Motor OK: StepPosition: %lld", currentStepPosition);

        if(controllerState != STATE_INIT && controllerState != STATE_NO_FEEDBACK && (ros::Time::now() - lastTeensyTime).toSec() > TEENSY_NODE_TIMEOUT){
          ROS_ERROR("Lost teensy connection!");
          controllerState = STATE_NO_FEEDBACK;
          // STOP
          CPhidgetStepper_setTargetPosition(steeringStepper, 0, currentStepPosition);
          CPhidgetStepper_setEngaged(steeringStepper, 0, 0);
        } else {
          if(controllerState == STATE_ERROR){
            if(hasCalibration){
              controllerState = STATE_OK;
            } else {
              controllerState = STATE_MISSING_CALIBRATION;
            }

          }
        }

    } else {
        ROS_ERROR("Phidget connection error");
        controllerState = STATE_ERROR;
    }

  switch (controllerState) {
    case STATE_INIT:
      ROS_INFO_THROTTLE(1,"Waiting for teensy data.");

      break;
    case STATE_MISSING_CALIBRATION:
      diagnose();
      ROS_WARN_THROTTLE(1,"Missing limit calibration.");
      break;

    case STATE_NO_FEEDBACK:
      ROS_WARN_THROTTLE(1,"Not recieving measurements or problems with data from teensy node!");
      break;

    case STATE_CALIBRATING_LIMITS:
      diagnose();
      calibrate();
      break;

    case STATE_CALIBRATING_VELOCITY:
      diagnose();
      calibrateVelocity();
      break;

    case STATE_OK:
    {
      diagnose();
      ROS_DEBUG_THROTTLE(1, "Encoder value: %f", encoderCount);
      ROS_DEBUG_THROTTLE(1, "Pot value: %f", potValue);

      double x = potToDisplacement(potLowpass.value());
      ROS_DEBUG_THROTTLE(1, "Displacement: %f", x);

      leftAngle = calcLeftAngle(x);
      rightAngle = calcRightAngle(x);
      ROS_DEBUG_THROTTLE(1, "Angles l,r: (%f, %f)", leftAngle, rightAngle);
      double dPot = potReference - potValue;
      int steps = -dPot*POT_TO_DISPLACEMENT/PINION_RADIUS * STEPPER_STEPS_PER_REVOLUTION/(2*M_PI);

      ROS_DEBUG_THROTTLE(1,"dPot: %f, required steps: %i", dPot, steps);

      if(abs(steps)>400 || !moving && abs(steps)>100){
        if(g1.res()){ // Two different detectable errors, right now they are "solved" in the same way
          // Motor is stalling, disengage
          steps = 0;
          diagInit = false;
          CPhidgetStepper_setEngaged(steeringStepper, 0, 0);
        } else if(g2.res()) {
          // Slip gear is slipping, "restart acceleration" by disengaging stepper
          steps = 0;
          diagInit = false;
          CPhidgetStepper_setEngaged(steeringStepper, 0, 0);
        } else {
          ROS_DEBUG_THROTTLE(1, "No errors, setting engaged: %s", engaged ? "true" : "false");
          CPhidgetStepper_setEngaged(steeringStepper, 0, engaged);

          if(engaged){
            ROS_DEBUG("Trying to move %i steps, target: %lld %s", steps, currentStepPosition+steps, !moving ? "(notmoving)" : "");
          }


        }

        // Adaptive turning speed
        if(vehicleVelocity > INCREASE_VELOCITY_THRESHOLD){
          currentStepperVelocity = stepperVelocity + VELOCITY_INCREASE_COEFFICIENT*K_UNIT * vehicleVelocity;
          if(currentStepperVelocity > MAX_VELOCITY) currentStepperVelocity = MAX_VELOCITY; // Limit

          ROS_DEBUG_THROTTLE(1, "Vehicle is moving, vel: %f, set velocity limit to %i", vehicleVelocity, currentStepperVelocity);
          CPhidgetStepper_setVelocityLimit(steeringStepper, 0, currentStepperVelocity);
        } else {
          currentStepperVelocity = stepperVelocity;
          ROS_DEBUG_THROTTLE(1, "Vehicle is NOT moving, vel: %f, set velocity limit to %i", vehicleVelocity, currentStepperVelocity);
          CPhidgetStepper_setVelocityLimit(steeringStepper, 0, currentStepperVelocity);
        }

        CPhidgetStepper_setAcceleration(steeringStepper, 0, TARGET_ACCELERATION);
        CPhidgetStepper_setTargetPosition(steeringStepper, 0, currentStepPosition+steps);
      }
      break;
    }

    case STATE_ERROR:
      diagnose();
      ROS_WARN("Controller in error state.");
      break;
  }

}

void SteeringSystem::diagnose(){
  //int stoppedState; // true when not moving and there are no outstanding commands
  //CPhidgetStepper_getStopped(steeringStepper, 0, &stoppedState);
  // moving = !stoppedState

  moving = !(lastStepPosition == currentStepPosition);
  //ROS_DEBUG("moving: %s", moving ? "yes" : "no");

  double stepsInRadians = currentStepPosition * 2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION;
  double encoderInRadians = (encoderCount)*(-1.0/0.375) * 2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION; // using encoder/step ratio 0.375 and opposite direction

  double d1 = stepsInRadians - encoderInRadians;

  double encoderInMm = -encoderInRadians*PINION_RADIUS;
  double potInMm = potValue*POT_TO_DISPLACEMENT;

  double d2 = encoderInMm - potInMm;

  if(g1.res()){
    ROS_WARN("Motor is stalling!");
    motorSkipping = true;
  } else {
    motorSkipping = false;
  }

  if(g2.res()){
    ROS_WARN("Slip gear is slipping!");
    gearSlipping = true;
  } else {
    gearSlipping = false;
  }

  // diagInit is true if the diagnosis system is already initialized. We can reinitialize by setting it to false elsewhere in the code
  if(diagInit){
    r1.update(d1);
    r2.update(d2);
  } else {
    r1.init(d1);
    r2.init(d2);
    g1.zero();
    g2.zero();
    diagInit = true;
  }

  //ROS_DEBUG("stepsInRadians: %f, encoderInRadians: %f, stepsInMm: %f, potInMm: %f", stepsInRadians, encoderInRadians, stepsInMm, potInMm);
  //ROS_DEBUG("d1: %f, d2: %f", d1, d2);
  //ROS_DEBUG("r1: %f, r2: %f", r1.value(), r2.value());
  g1.calc(r1.value());
  g2.calc(r2.value());
  //ROS_DEBUG("g1: %f,%f, g2: %f,%f", g1.valueplus(), g1.valueminus(), g2.valueplus(), g2.valueminus());

  #if LOGGING
  logfile << stepsInRadians << " " << encoderInRadians << " " << encoderInMm << " " << potInMm << " " << d1 << " " << d2 << " " << r1.value() << " " << r2.value() << " " << g1.valueplus() << " " << g1.valueminus() << " " << g2.valueplus() << " " << g2.valueminus() << "\n";
  #endif

  lastStepPosition = currentStepPosition;
}

void SteeringSystem::calibrate(){
  // Go all the way to each side to find min and max on potentiometer.

  switch (calibrationState) {
    case GOING_LEFT:

      // Keep going left until stepper is missing steps or slip gear is slipping
      if(g1.res() || g2.res()){
        potMin = 1.05*potLowpass.value();
        calibrationState = GOING_RIGHT;

        g1.zero();
        g2.zero();

        ROS_INFO("Finished going left.. going right..");
        startTurnTime = ros::Time::now();
      } else {
        int steps = 10.0/PINION_RADIUS * STEPPER_STEPS_PER_REVOLUTION/(2.0*M_PI); // command 1cm at a time
        CPhidgetStepper_setAcceleration(steeringStepper, 0, CALIBRATION_ACCELERATION);
        CPhidgetStepper_setVelocityLimit(steeringStepper, 0, CALIBRATION_VELOCITY);
        CPhidgetStepper_setTargetPosition(steeringStepper, 0, currentStepPosition+steps);
        CPhidgetStepper_setEngaged(steeringStepper, 0, 1);
      }
      break;

    case GOING_RIGHT:
      // Keep going right until stepper is missing steps or slip gear is slipping
      if((g1.res() || g2.res())){
        if((ros::Time::now()-startTurnTime).toSec() > 1.0){
         // sometimes it is still detecting the previous limit, so add a timeout
        potMax = 0.95*potLowpass.value();
        if(potMax - potMin < 0.3*(POTENTIOMETER_MAX-POTENTIOMETER_MIN)){
          // Something went wrong in calibration, difference between limits is too small
          ROS_WARN("Limit calibration failed!");

          if(hasCalibration){
            // If we already had a calibration, keep using that
            ROS_WARN("Reloading previous calibration");
            bool res = loadConfig();
            if(res){
              controllerState = STATE_OK;
            } else {
              ROS_WARN("Error reloading old config! Recalibration needed");
              controllerState = STATE_MISSING_CALIBRATION;
            }

          } else {
            controllerState = STATE_MISSING_CALIBRATION;
          }

          g1.zero();
          g2.zero();

        } else {
          potZero = 0.5*(potMin+potMax);
          saveConfig();
          controllerState = STATE_OK;
          hasCalibration = true;
          ROS_INFO("Calibration finished, min: %f, max: %f, zero: %f", potMin, potMax, potZero);

          // update mu1
          g1.setParams(0, TARGET_VELOCITY*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION, 35,50);
          g2.setParams(0, TARGET_VELOCITY*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION*PINION_RADIUS, 4.7, 100);

          CPhidgetStepper_setEngaged(steeringStepper, 0, 0);
          g1.zero();
          g2.zero();
          potReference = displacementToPot(calcDisplacement(angleReference));
        }
        } else {
          ROS_INFO("Detected end but in timeout");
        }



      } else {
        //ROS_INFO_THROTTLE(1, "going right..");
        int steps = -10.0/PINION_RADIUS * STEPPER_STEPS_PER_REVOLUTION/(2.0*M_PI); // 1cm at a time
        CPhidgetStepper_setAcceleration(steeringStepper, 0, CALIBRATION_ACCELERATION);
        CPhidgetStepper_setVelocityLimit(steeringStepper, 0, CALIBRATION_VELOCITY);
        CPhidgetStepper_setTargetPosition(steeringStepper, 0, currentStepPosition+steps);
        CPhidgetStepper_setEngaged(steeringStepper, 0, 1);
      }

      break;
  }

}

void SteeringSystem::calibrateVelocity(){
  int steps = 0;
  if(calibrationState == GOING_RIGHT){
    double dPot = 0.9*potMax - potValue;

    if(abs(dPot) < 5.0){
      timesWithoutSlip++;
      ROS_INFO("Turned %i times without stalling.", timesWithoutSlip);
      calibrationState = GOING_LEFT;
    } else {
      // Still go right
      if(!moving){
        steps = -dPot*POT_TO_DISPLACEMENT/PINION_RADIUS * STEPPER_STEPS_PER_REVOLUTION/(2*M_PI);
        ROS_INFO("Commanding %i steps", steps);
      }

    }

  } else {
    double dPot = 1.1*potMin - potValue;

    if(abs(dPot) < 5.0){
      timesWithoutSlip++;
      ROS_INFO("Turned %i times without stalling.", timesWithoutSlip);
      calibrationState = GOING_RIGHT;
    } else {

      if(!moving){
        steps = -dPot*POT_TO_DISPLACEMENT/PINION_RADIUS * STEPPER_STEPS_PER_REVOLUTION/(2*M_PI);
        ROS_INFO("Commanding %i steps", steps);
      }

    }

  }

  // Check if we are slipping
  // If we are, reduce speed
  if(g1.res() || g2.res()){
    ROS_INFO("Stalled, resetting counter...");
    timesWithoutSlip = 0;
    diagInit = false;
    CPhidgetStepper_setEngaged(steeringStepper, 0, 0);
    stepperVelocity *= 0.95;
    ROS_INFO("Reduced stepper velocity to: %i", stepperVelocity);
    g1.setParams(0,stepperVelocity*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION, 0.01,500);
    g2.setParams(0,stepperVelocity*2.0*M_PI/STEPPER_STEPS_PER_REVOLUTION*PINION_RADIUS,1.0,350);
    g1.zero();
    g2.zero();

  } else {

    if(abs(steps)>0){
      CPhidgetStepper_setAcceleration(steeringStepper, 0, TARGET_ACCELERATION);
      CPhidgetStepper_setVelocityLimit(steeringStepper, 0, stepperVelocity);
      CPhidgetStepper_setTargetPosition(steeringStepper, 0, currentStepPosition+steps);
      CPhidgetStepper_setEngaged(steeringStepper, 0, 1);
    }
    }

  // Check if we are done, exit velocity calibration
  if(timesWithoutSlip >= 4){
    ROS_INFO("Steering velocity calibration finished.");
    controllerState = STATE_OK;
    saveConfig();
  }

}

void SteeringSystem::sanityCheck(){
  // Check math heavy functions for our Ackermann geometry
  double leftAngle = calcLeftAngle(0.0531);
  double rightAngle = calcRightAngle(0.0531);
  //ROS_INFO("Left: %f, Right: %f", leftAngle, rightAngle);
  double xl = calcDisplacementLeft(leftAngle);
  double xr = calcDisplacementRight(rightAngle);
  //ROS_INFO("xl: %f, xr: %f", xl, xr);

  if(abs(xl-0.0531) > 0.0001){
    ROS_WARN("Steering node has gone insane, displ<->angle");
  }
  if(abs(xr-0.0531) > 0.0001){
    ROS_WARN("Steering node has gone insane, displr<->angle");
  }

  // Check other conversions
  if(abs(potToDisplacement(displacementToPot(0.123)))-0.123 > 0.00001){
    ROS_WARN("Steering node has gone insane, pot<->disp");
  }
}

bool SteeringSystem::checkPotentiometerValue(double value){
  return (value >= POTENTIOMETER_MIN && value <= POTENTIOMETER_MAX);
}

double SteeringSystem::potToDisplacement(double pot){
  return (pot-potZero)*POT_TO_DISPLACEMENT;
}

double SteeringSystem::displacementToPot(double x){
  return x/POT_TO_DISPLACEMENT+potZero;
}

double SteeringSystem::calcRightAngle(double x){
  double d = 0.5*(WIDTH-RACK_LENGTH) - x;
  double r = sqrt(d*d + OFFSET*OFFSET);

  return 180*( asin(d/r) - acos( ( STEERING_ARM_LENGTH*STEERING_ARM_LENGTH+r*r-TIE_ROD_LENGTH*TIE_ROD_LENGTH )/( 2.0*STEERING_ARM_LENGTH*r ) ) )/M_PI - thetaZero;
}

double SteeringSystem::calcLeftAngle(double x){
  // Symmetry
  return -calcRightAngle(-x);
}

double SteeringSystem::calcDisplacement(double referenceAngle){
  // Reference angle in degrees
  const double tanSteer = tan(M_PI*referenceAngle/180);
  ROS_INFO("Commanded steering angle: %f deg", referenceAngle);

  // Calculate ideal Ackermann steering angles
  // double ackermannLeft  = 180*atan2(tanSteer,1-FRONT_TRACK_WIDTH/(2*WHEEL_BASE)*tanSteer)/M_PI;
  // double ackermannRight = 180*atan2(tanSteer,1+FRONT_TRACK_WIDTH/(2*WHEEL_BASE)*tanSteer)/M_PI;
  // ROS_INFO("Ideal Ackermann angles: %f, %f", ackermannLeft, ackermannRight);
  //
  // double sleft = calcDisplacementLeft(ackermannLeft);
  // double sright = calcDisplacementRight(ackermannRight);

  // Simplification, much simpler and with negligible difference
  double sleft = calcDisplacementLeft(referenceAngle);
  double sright = calcDisplacementRight(referenceAngle);

  double res = 0.5*(sleft+sright);
  ROS_INFO("Required rack displacement: %f mm", res);

  ROS_INFO("Will result in actual angles: %f, %f", calcLeftAngle(res), calcRightAngle(res));

  return res;
}

double SteeringSystem::calcDisplacementRight(double rightAngle){
  double theta = M_PI*(rightAngle+thetaZero)/180.0;
  return 0.5*(WIDTH-RACK_LENGTH) - (STEERING_ARM_LENGTH*sin(theta) + sqrt(TIE_ROD_LENGTH*TIE_ROD_LENGTH-STEERING_ARM_LENGTH*STEERING_ARM_LENGTH*cos(theta)*cos(theta)-OFFSET*OFFSET+2*STEERING_ARM_LENGTH*OFFSET*cos(theta)));
}

double SteeringSystem::calcDisplacementLeft(double leftAngle){
  return -calcDisplacementRight(-leftAngle);
}

double SteeringSystem::getLeftAngle(){
  return leftAngle;
}
double SteeringSystem::getRightAngle(){
  return rightAngle;
}

double SteeringSystem::getSteeringSpeed(){
  return currentStepperVelocity*stepsToAngleSpeedCoefficient;
}

bool SteeringSystem::isSkipping(){
  return motorSkipping;
}

bool SteeringSystem::isSlipping(){
  return gearSlipping;
}

void SteeringSystem::close(){
  CPhidgetStepper_setEngaged(steeringStepper, 0, 0);
  CPhidget_close((CPhidgetHandle)steeringStepper);
  CPhidget_delete((CPhidgetHandle)steeringStepper);

  #if LOGGING
  logfile.close();
  #endif
}
