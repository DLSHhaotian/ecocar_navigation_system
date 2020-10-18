#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Trigger.h>
#include <dynamo_msgs/TeensyWrite.h>
#include <dynamo_msgs/TeensyRead.h>
#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <serial_tools.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#define DEBUG_SEND 0
#define DEBUG_READ 1

#define TEENSY_LOOP_RATE 100   // Maximum rate ~600
#define BAUDRATE 1000000
#define MAX_RX_PACKET_SIZE 400
#define MAX_TX_PACKET_SIZE 100
#define SEND_TIMEOUT 20     // [ms]
#define READ_TIMEOUT 20     // [ms]
#define TEENSY_WRITE_TIMEOUT 1 // [s]

enum NodeState {
    STATE_FATAL_ERROR = -1, // For unrecoverable errors
    STATE_OK,               // When OK
    STATE_WAIT              // Node is not ready or trying to recover from an error
};

void connectSerial();
inline bool syncSerial();

NodeState state;
ros::Publisher teensyPub;
struct sp_port *port;
char readBuffer[MAX_RX_PACKET_SIZE];
float resetDistanceOffset;
ros::Time lastTeensyWriteTime;

Json::Reader reader;
Json::FastWriter writer;
Json::Value sendValue;
Json::Value receiveValue;

void cmdTeensyCallback(const dynamo_msgs::TeensyWritePtr& msg) {
    lastTeensyWriteTime = ros::Time::now();
    sendValue["b"] = msg->burn ? 1 : 0;
    sendValue["i"] = msg->light ? 1 : 0;
    sendValue["l"] = msg->blink_left ? 1 : 0;
    sendValue["r"] = msg->blink_right ? 1 : 0;
    sendValue["h"] = msg->horn ? 1 : 0;
    sendValue["p"] = msg->parking ? 1 : 0;
    sendValue["a"] = msg->autopilot_active ? 1 : 0;
}

bool aliveCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    res.success = (state == STATE_OK);
    return true;
}

int main(int argc, char *argv[]) {
    state = STATE_WAIT;
	ros::init(argc, argv, "teensy_node");

    // Configure logger
#if (DEBUG_READ || DEBUG_SEND)
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
#endif

	ROS_DEBUG("Starting Teensy Driver. Searching for Teensy...");

	ros::NodeHandle nh;

    // Initial values
    resetDistanceOffset = -1;
    sendValue["b"] = 0;
    sendValue["i"] = 1;
    sendValue["l"] = 1;
    sendValue["r"] = 1;
    sendValue["h"] = 0;
    sendValue["p"] = 0;
    sendValue["a"] = 0;

    ros::Subscriber teensySub = nh.subscribe("teensy_write", 1, cmdTeensyCallback);
    ros::ServiceServer service = nh.advertiseService("teensy_alive", aliveCallback);
    teensyPub = nh.advertise<dynamo_msgs::TeensyRead>("teensy_read", 1);

    connectSerial();

    ros::Rate loopRate(TEENSY_LOOP_RATE);

    bool success;
    while(ros::ok()) {
        ros::spinOnce();

        success = syncSerial();
        if(success) {
            state = STATE_OK;
            ROS_INFO_THROTTLE(10, "Teensy data parsing successfully...");
        } else {
            state = STATE_WAIT;
        }

        loopRate.sleep();
    }
    ROS_INFO("Closing Teensy port");
    sp_close(port);
    std::exit(EXIT_SUCCESS);
}

void connectSerial() {
    state = STATE_WAIT;
    // Set up serial connection
    char* portName = NULL;
    while(ros::ok() && portName == NULL) {
        ROS_INFO_THROTTLE(5, "Waiting for Teensy connection...");
        portName = get_port_name(TEENSY_SERIAL_NUMBER);
        ros::spinOnce();
        usleep(1000);   // Wait 1 ms
    }
    ROS_INFO("Opening Teensy on port '%s'", portName);
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
    state = STATE_OK;
}

inline bool syncSerial() {
    dynamo_msgs::TeensyRead msg;

    // Send JSON
    receiveValue.clear();
    if((ros::Time::now() - lastTeensyWriteTime).toSec() > TEENSY_WRITE_TIMEOUT) {
        sendValue["a"] = 0;
    }
    std::string sendString = writer.write(sendValue);
#if DEBUG_SEND
    ROS_DEBUG("Send: %s", sendString.c_str());
#endif
    sp_return spReturn = sp_blocking_write(port, sendString.c_str(), sendString.length(), SEND_TIMEOUT);
    if(spReturn < 0) {
        ROS_ERROR("Serial error %d during write: %s", spReturn, sp_last_error_message());
        connectSerial();
        return false;
    }
    int i = 0;
    while(sp_output_waiting(port) > 0) {
        i += 10;
        usleep(10);
        if(i > (SEND_TIMEOUT * 1000)) {
            ROS_ERROR("Send to Teensy timeout");
            return false;
        }
    }

    // Decode JSON
    readBuffer[0] = '\0';
    for(int i = 0; i < MAX_RX_PACKET_SIZE; i++) {
        spReturn = sp_blocking_read(port, &readBuffer[i], 1, READ_TIMEOUT);
        if(spReturn < 0) {
            ROS_ERROR("Serial error %d during read: %s", spReturn, sp_last_error_message());
            connectSerial();
            return false;
        }
        if(readBuffer[i] == '}') break;
    }
    if(sp_input_waiting(port) > 0) {
        ROS_WARN("Waiting input %d", sp_input_waiting(port));
        sp_flush(port, SP_BUF_INPUT);
    }
    int endTagIndex = -1;
    for(int i = 0; i < MAX_RX_PACKET_SIZE && readBuffer[i] != '\0'; i++) {
        if(readBuffer[i] == '}') {
            endTagIndex = i;
            break;
        }
    }
    char * readBufferPtr = strstr(readBuffer, "{");
    if(readBufferPtr == NULL || readBufferPtr[0] != '{' || endTagIndex == -1) {
        ROS_WARN("Discarded incomplete or invalid data");
        ROS_WARN("Data: <<%s>>", readBuffer);
        return false;
    } else if(endTagIndex < MAX_RX_PACKET_SIZE - 2) {
        readBufferPtr[endTagIndex + 1] = '\0';
    }

    bool parsingSuccessful = reader.parse(readBufferPtr, receiveValue);
    if(!parsingSuccessful) {
        ROS_WARN("Failed to parse configuration: %s", reader.getFormattedErrorMessages().c_str());
        ROS_WARN("Data: <<%s>>", readBufferPtr);
        return false;
    }
    msg.header.stamp = ros::Time::now();
    msg.speed_wheel = receiveValue.get("sW", -1).asFloat();
    msg.brake_pressure_1 = receiveValue.get("p1", -1).asFloat();
    msg.brake_pressure_2 = receiveValue.get("p2", -1).asFloat();
    msg.brake_encoder = receiveValue.get("bE", -1).asFloat();
    msg.steering_encoder = receiveValue.get("sE", -1).asFloat();
    msg.steering_potentiometer = receiveValue.get("sP", -1).asFloat();
    msg.gear = receiveValue.get("gr", -1).asFloat();
    if(resetDistanceOffset == -1) resetDistanceOffset = receiveValue.get("dW", -1).asFloat();   // TODO Maybe service based reset?
    msg.distance_wheel = receiveValue.get("dW", -1).asFloat() - resetDistanceOffset;
    msg.distance_front = receiveValue.get("dF", -1).asFloat();
    msg.distance_left = receiveValue.get("dL", -1).asFloat();
    msg.distance_right = receiveValue.get("dR", -1).asFloat();
    msg.autopilot = receiveValue.get("aP", -1).asFloat();
    msg.batteryVoltage = receiveValue.get("bV", -1).asFloat();
	msg.engineBatteryVoltage = receiveValue.get("eV", -1).asFloat();
	msg.emergencyButton = receiveValue.get("eB", -1).asFloat();

    if(msg.speed_wheel == -1.0) {
	ROS_WARN("Teensy went -1!");
	return false;
	}

    teensyPub.publish(msg);

#if DEBUG_READ
    ROS_DEBUG("Read: %s", readBuffer);
#endif
    return true;
}
