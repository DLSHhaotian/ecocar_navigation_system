#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <cstdlib>
#include <stdio.h>
#include <libserialport.h> 
#include <serial_tools.h>

#define BAUDRATE 115200
#define PACKET_SIZE 8
#define GYRO_LOOP_RATE 100
#define READ_TIMEOUT 10

enum NodeState {
    NODE_STATE_FATAL_ERROR = -1, // For unrecoverable errors
    NODE_STATE_OK,               // When OK
    NODE_STATE_WAIT              // Node is not ready or trying to recover from an error
};

inline bool parseSerial();

NodeState state;
ros::Publisher gyroPub;
struct sp_port *port;
unsigned char readBuffer[PACKET_SIZE];
float resetAngleOffset;

void connectSerial() {
    state = NODE_STATE_WAIT;
    // Set up serial connection
    char* portName = NULL;
    while(ros::ok() && portName == NULL) {
        portName = get_port_name(GYRO_SERIAL_NUMBER);
        ros::spinOnce();
        usleep(1000);   // Wait 1 ms
        ROS_INFO_THROTTLE(5, "Waiting for Gyro connection...");
    }
    ROS_INFO("Opening Gyro on port '%s' \n", portName);
    sp_return error = sp_get_port_by_name(portName, &port);
    if(error == SP_OK) {
        error = sp_open(port, SP_MODE_READ_WRITE);
        if(error == SP_OK) {
            sp_set_baudrate(port, BAUDRATE);
        } else {
            ROS_ERROR("Error opening serial device: %d, %s", error, sp_last_error_message());
            sp_close(port);
            usleep(1000000);    // Wait 1 s
            // Recursion!
            if(ros::ok()) connectSerial();
        }
    } else {
        ROS_ERROR("Error finding serial device: %d, %s", error, sp_last_error_message());
        sp_close(port);
        usleep(1000000);        // Wait 1 s
        // Recursion!
        if(ros::ok()) connectSerial();
    }
    state = NODE_STATE_OK;
}

bool aliveCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if(!req.data) {
        ROS_INFO("Shutdown triggered by service!");
        if(state == NODE_STATE_WAIT) std::exit(EXIT_SUCCESS);
        state = NODE_STATE_FATAL_ERROR;
    }
    res.success = (state == NODE_STATE_OK);
    return true;
}

int main(int argc, char *argv[]) {
    state = NODE_STATE_WAIT;
	ros::init(argc, argv, "gyro_node");
	ROS_DEBUG("Starting Gyro Driver");

	ros::NodeHandle nh;
    resetAngleOffset = -1;

    gyroPub = nh.advertise<std_msgs::Float32>("gyro_angle", 1);
    ros::ServiceServer service = nh.advertiseService("gyro_alive", aliveCallback);

    connectSerial();

    ros::Rate loopRate(GYRO_LOOP_RATE);

    while(ros::ok()) {
        if(state == NODE_STATE_FATAL_ERROR) break;
        bool success = parseSerial();
        if(success) state = NODE_STATE_OK;
        else state = NODE_STATE_WAIT;

        ros::spinOnce();
        loopRate.sleep();
    }  

    sp_close(port);

    std::exit(EXIT_SUCCESS);
}

inline bool parseSerial() {
    std_msgs::Float32 msg;
    sp_return spReturn;
    short header;
    short rateInt;
    short angleInt;
    float rateFloat;
    float angleFloat;
    short check_sum;
	int startBytesReceived = 0;
	
	readBuffer[0] = '\0';
    for(int i = 0; i < PACKET_SIZE && ros::ok(); i++) {
        spReturn = sp_blocking_read(port, &readBuffer[i], 1, READ_TIMEOUT);
        if(spReturn < 0) {
            ROS_ERROR("Serial error %d during read: %s", spReturn, sp_last_error_message());
            connectSerial();
            return false;
        }
        if(readBuffer[i] == 0xFF && startBytesReceived < 2) {
			if(startBytesReceived == 0) {
				startBytesReceived = 1;
			} else if(startBytesReceived == 1) {
                startBytesReceived = 2;
				if(i != 1) {
                    ROS_WARN("Data lost! Continuing...");
                    readBuffer[0] = 0xFF;
                    readBuffer[1] = 0xFF;
                    i = 1;
                }
			}
		}
    }
    sp_flush(port, SP_BUF_INPUT);

    // Copy values from data string
    memcpy(&rateInt, readBuffer+2, sizeof(short));
    memcpy(&angleInt, readBuffer+4, sizeof(short));
    memcpy(&check_sum, readBuffer+6, sizeof(short));

    // Verify checksum
    if(check_sum != (short)(0xFFFF + rateInt + angleInt)) {
        ROS_WARN("Checksum error");
        return false;
    }

    // Apply scale factors
    rateFloat = rateInt / 100.0;
    angleFloat = angleInt / 100.0;

    if(resetAngleOffset == -1) resetAngleOffset = angleFloat;
    angleFloat = angleFloat - resetAngleOffset;

    msg.data = angleFloat;
    gyroPub.publish(msg);

    ROS_INFO("Gyro Rate: %f, [deg/sec]\tangleFloat: %f [deg]", rateFloat, angleFloat);
    return true;
}