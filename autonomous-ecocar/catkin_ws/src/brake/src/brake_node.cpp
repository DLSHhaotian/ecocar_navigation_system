#include "BrakeSystem.h"
#include "BrakeSystem.cpp"

#define DEBUG_LOGGER 1

#define CONTROL_LOOP_RATE 20 // Pololu controller runs at 20Hz as well (with higher frequency output signal becomes too noisy)

BrakeSystem brakeSystem(CONTROL_LOOP_RATE);

enum NodeState {
    NODE_STATE_FATAL_ERROR = -1, // For unrecoverable errors
    NODE_STATE_OK,				 // When OK
    NODE_STATE_CALIBRATE,               
    NODE_STATE_WAIT,              // Node is not ready or trying to recover from an error
    NODE_STATE_TEST
};

NodeState state;

float oldpower = 0;

bool StartCalibration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	state = NODE_STATE_CALIBRATE;
}

// Subscriber Callbacks
void brakePowerCallback(const dynamo_msgs::BrakeStepperPtr& msg) {
    ROS_INFO("Brake heard: [%f] %s", msg->brake_power, msg->brake_stepper_engaged ? "engaged" : "disengaged");
	brakeSystem.engage = msg->brake_stepper_engaged;
	brakeSystem.power = msg->brake_power;
	// Update the reference of the control loop
    	// TODO: updateReference(msg->brake_power, msg->brake_stepper_engaged)
}

void teensyCallback(const dynamo_msgs::TeensyReadPtr& msg) {
    // Update the brake motor control (Note: encoderOffset is added in updateFeedback function)
    brakeSystem.UpdateRef(msg->brake_encoder, msg->brake_pressure_1, msg->brake_pressure_2);
}

bool aliveCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if(!req.data) {
        ROS_INFO("Shutdown triggered by service!");
        state = NODE_STATE_FATAL_ERROR;
    }
    res.success = (state == NODE_STATE_OK);
    return true;
}

// MAIN //
int main(int argc, char *argv[]) {
  state = NODE_STATE_WAIT;
	ros::init(argc, argv, "brake_node");
	ros::NodeHandle nh;
    
	// Comment test
	// Configure logger
  #if(DEBUG_LOGGER)
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  #endif

  // Prepare ROS
  ros::Subscriber brakeSub = nh.subscribe("cmd_brake_power", 1, brakePowerCallback);
  ros::Subscriber teensySub = nh.subscribe("teensy_read", 1, teensyCallback);

  ros::ServiceServer service = nh.advertiseService("brake_alive", aliveCallback);
  ros::ServiceServer calibrate = nh.advertiseService("Brake_Calibrate", StartCalibration);

	
  // ROS spin once to get initial callbacks
  ros::spinOnce();

  // Start control loop timer
  ros::Rate rate(CONTROL_LOOP_RATE);
  while(ros::ok()) {
    ros::spinOnce();

    if(brakeSystem.press1 <= -1 || brakeSystem.press1 > 34)
    {
      ROS_ERROR("Pressure sensor 1 out of bounds!");
      ROS_INFO("Pressure measured to: %f\n",brakeSystem.press1);
      brakeSystem.SentToCon(brakeSystem.position);
      state = NODE_STATE_WAIT;
    }
    else
    {
      state = NODE_STATE_OK;
    }
    if(brakeSystem.press2 <= -1 || brakeSystem.press2 > 34)
    {
      ROS_ERROR("Pressure sensor 2 out of bounds!");
      ROS_INFO("Pressure measured to: %f\n",brakeSystem.press2);
      brakeSystem.SentToCon(brakeSystem.position);
      state = NODE_STATE_WAIT;
    }
    else
    {
      state = NODE_STATE_OK;
    }
    if(brakeSystem.pot < 0 || brakeSystem.pot > 5)
    {
      ROS_ERROR("Potentiometer reading out of bounds!");
      brakeSystem.SentToCon(brakeSystem.position);
      state = NODE_STATE_WAIT;
    }
    else
    {
      state = NODE_STATE_OK;
    }

    if(state == NODE_STATE_FATAL_ERROR){
      ROS_ERROR("Brake node going down");
      break;
    }
    //state = NODE_STATE_TEST;

    float test = 0;
    float tests = 0;
    brakeSystem.timers = brakeSystem.timers + 1;
    if(state == NODE_STATE_TEST)
    {
      ROS_INFO("Pressure measured to: %f\n",brakeSystem.press2);
      ROS_INFO("Position measured to: %f\n",brakeSystem.pot);
      ROS_INFO("Position set to: %f\n",brakeSystem.position);
      if(brakeSystem.timers < 1000)
      {
        brakeSystem.BrakePID(test);
        brakeSystem.SentToCon(brakeSystem.position);
      }
      if(brakeSystem.timers > 1000)
      {
        brakeSystem.SentToCon(brakeSystem.posMin);
      }
	}

    
  if(state == NODE_STATE_CALIBRATE)
	{
        ROS_INFO("Pressure measured to: %f\n",brakeSystem.press2);
        ROS_INFO("Position measured to: %f\n",brakeSystem.pot);
        ROS_INFO("Position set to: %f\n",brakeSystem.position);
        ROS_INFO("PosMax set to: %f\n",brakeSystem.posMax);
        ROS_INFO("PosMin set to: %f\n",brakeSystem.posMin);
        brakeSystem.timers = brakeSystem.timers + 1;
        bool CalOK = brakeSystem.CalibrateBrake();
        brakeSystem.SentToCon(brakeSystem.position);
		if(CalOK)
		{
			state = NODE_STATE_OK;
		}
	}

	if(state == NODE_STATE_OK)
	{	      
    ROS_INFO("state ok");
    if(brakeSystem.power != oldpower)
    {
      //brakeSystem.integral = 0; // Not desirable: this means that when the reference pressure is changed the integral is reset!
    }
    oldpower = brakeSystem.power;
    if(brakeSystem.engage == true)
    {
		ROS_INFO("Position set to: %f\n",brakeSystem.position);
		ROS_INFO("Position measured to: %f\n",brakeSystem.pot);
		ROS_INFO("Pressure level set to bar: %f\n",brakeSystem.power);
		ROS_INFO("Pressure measured to: %f\n",brakeSystem.press2);
      if(brakeSystem.power == 0)
      {
        brakeSystem.SentToCon(brakeSystem.posMin);
		brakeSystem.integral = 0; // When the reference is 0 bar reset the integrator
		brakeSystem.pre_integral = 0;
      }
      else
      {
        brakeSystem.BrakePID(brakeSystem.power);
        brakeSystem.SentToCon(brakeSystem.position);
      }
    }
    else
    {
		brakeSystem.SentToCon(brakeSystem.posMin);
		brakeSystem.integral = 0; // When the brakes are disengaged reset integrator such that it will start from zero when engaged
		brakeSystem.pre_integral = 0;
    }
	}
    
    rate.sleep();

  }
  brakeSystem.close();
  // Close down connection
  return 0;
}
