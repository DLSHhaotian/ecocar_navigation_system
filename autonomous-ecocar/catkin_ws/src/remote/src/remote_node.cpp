#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/NavSatFix.h>
#include <dynamo_msgs/Speak.h>
#include <dynamo_msgs/BrakeStepper.h>
#include <dynamo_msgs/SteeringStepper.h>
#include <dynamo_msgs/TeensyWrite.h>
#include <dynamo_msgs/TeensyRead.h>
#include <stdio.h>
#include <string>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#define REMOTE_LOOP_RATE 10
#define REMOTE_PORT_NUMBER 57994
#define MAX_RX_MESSAGE_SIZE 256
#define REMOTE_TIMEOUT 1        // [s]
#define DEBUG_LOGGER 1
#define NOTHING_RECEIVED_IN_A_ROW_THRESHOLD 100

#define MAX_BRAKE_PRESSURE 10	// [bar]
#define MAX_STEERING_ANGLE 20	// [deg]

ros::Time lastMessageReceivedTime;

enum NodeState {
    STATE_FATAL_ERROR = -1, // For unrecoverable errors
    STATE_OK,               // When OK
    STATE_WAIT              // Node is not ready or trying to recover from an error
};

bool connectRemote();
void closeRemote();

NodeState state;
ros::Publisher teensyPub;
ros::Publisher steeringPub;
ros::Publisher brakePub;

int fd, sockfd, newsockfd;
int nothingReceivedInARow;
Json::Reader reader;
Json::FastWriter writer;
Json::Value sendValue;
Json::Value receiveValue;

void teensyCallback(const dynamo_msgs::TeensyReadPtr& msg) {
	sendValue["t"] = msg->millis;
    sendValue["sW"] = msg->speed_wheel;
    //sendValue["tW"] = msg->temperature_water;
    //sendValue["tO"] = msg->temperature_oil;
    //sendValue["lb"] = msg->lambda;
    sendValue["gr"] = msg->gear;
    sendValue["dW"] = msg->distance_wheel;
    //sendValue["dL"] = msg->distance_left;
    //sendValue["dR"] = msg->distance_right;
}

void gpsCallback(const sensor_msgs::NavSatFixPtr& msg) {
    sendValue["la"] = msg->latitude;
    sendValue["lo"] = msg->longitude;
	sendValue["al"] = msg->altitude;
    sendValue["gs"] = msg->status.status;
}

bool aliveCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("Remote is alive");
    res.success = (state == STATE_OK);
    return true;
}

int main(int argc, char *argv[]) {
    state = STATE_WAIT;
	ros::init(argc, argv, "remote_node");
	ROS_INFO("Starting Remote Node");

    system("~/Scripts/register_ip.sh");

    // Register to ROS
	ros::NodeHandle nh;
    ros::Subscriber gpsSub = nh.subscribe("gps/fix", 1, gpsCallback);
    ros::Subscriber teensySub = nh.subscribe("teensy_read", 1, teensyCallback);
    ros::ServiceServer service = nh.advertiseService("remote_alive", aliveCallback);
    teensyPub = nh.advertise<dynamo_msgs::TeensyWrite>("teensy_write", 1);
    steeringPub = nh.advertise<dynamo_msgs::SteeringStepper>("cmd_steering_angle", 1);
    brakePub = nh.advertise<dynamo_msgs::BrakeStepper>("cmd_brake_power", 1);

#if (DEBUG_LOGGER)
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
#endif

    // Open server
	char readBuffer[MAX_RX_MESSAGE_SIZE];
	int returnBytes;
    bool success = connectRemote();
    if(!success) {
        // TODO Handle
        ROS_ERROR("Remote connection unsuccesful");
        std::exit(EXIT_SUCCESS);
    }
    nothingReceivedInARow = 0;

    // Loop
    ros::Rate rate(REMOTE_LOOP_RATE);
    while(ros::ok()) {
        rate.sleep();
        dynamo_msgs::TeensyWrite teensyMsg;
		dynamo_msgs::SteeringStepper steeringMsg;
		dynamo_msgs::BrakeStepper brakeMsg;

        ros::spinOnce(); 
        // Burn safety
        /*if(ros::Time::now() - lastMessageReceivedTime > ros::Duration(REMOTE_TIMEOUT)) {
            teensyMsg.burn = false;
            teensyPub.publish(teensyMsg);
            brakeMsg.brake_stepper_engaged = true;
            brakeMsg.brake_power = 0.8;
            brakePub.publish(brakeMsg);
            ROS_ERROR("Connection timeout. Burn = false and brake = 0.8");
        }*/
        // Read message
        receiveValue.clear();
        bzero(readBuffer, MAX_RX_MESSAGE_SIZE);
        ROS_DEBUG("Reading message...");
		returnBytes = read(newsockfd, readBuffer, MAX_RX_MESSAGE_SIZE - 1);
		if(returnBytes < 0) {
            ROS_ERROR("ERROR reading from socket");
            // Connection probably lost. Reopen server.
	        closeRemote();
            bool success = connectRemote();
            if(!success) {
                // TODO Handle
                ROS_ERROR("Remote connection unsuccesful");
                std::exit(EXIT_SUCCESS);
            }
            nothingReceivedInARow = 0;
        } else if(returnBytes == 0) {
            ROS_INFO("Nothing received");
            nothingReceivedInARow++;
            if(nothingReceivedInARow > NOTHING_RECEIVED_IN_A_ROW_THRESHOLD) {
                ROS_WARN("Nothing received in a row above threshold. Trying to reconnect...");
                closeRemote();
                usleep(5000000);    // Wait 5 s
                bool success = connectRemote();
                if(!success) {
                    // TODO Handle (try reconnect continuously?)
                    ROS_ERROR("Remote connection unsuccesful");
                    std::exit(EXIT_SUCCESS);
                }
            }
        }
		ROS_INFO("Remote message received: %s\n", readBuffer);
        // Parse message
        int endTagIndex = -1;
        for(int i = 0; i < MAX_RX_MESSAGE_SIZE && readBuffer[i] != '\0'; i++) {
            if(readBuffer[i] == '}') {
                endTagIndex = i;
                break;
            }
        }
        char * readBufferPtr = strstr(readBuffer, "{");
        if(readBufferPtr == NULL || readBufferPtr[0] != '{' || endTagIndex == -1) {
            ROS_WARN("Discarded incomplete or invalid data");
            ROS_WARN("Data: <<%s>>", readBuffer);
            state = STATE_WAIT;
            continue;
        } else if(endTagIndex < MAX_RX_MESSAGE_SIZE - 2) {
            readBufferPtr[endTagIndex + 1] = '\0';
        }

        bool parsingSuccessful = reader.parse(readBufferPtr, receiveValue);
        if(!parsingSuccessful) {
            ROS_WARN("Failed to parse configuration: %s", reader.getFormattedErrorMessages().c_str());
            ROS_WARN("Data: <<%s>>", readBufferPtr);
            state = STATE_WAIT;
            continue;
        }
        ROS_INFO("Parse success");
        uint64_t time = receiveValue.get("t", 0).asUInt64();
		
		// Steering Angle is received as a percentage value (-100% -> 100%)
        steeringMsg.steering_angle = receiveValue.get("steeringP", 0).asFloat() / 100.0f * -1 * MAX_STEERING_ANGLE;
        steeringMsg.steering_stepper_engaged = true;
		steeringPub.publish(steeringMsg);
		// Brake Power is received as a percentage value (0% -> 100%)
        brakeMsg.brake_power = receiveValue.get("brakeP", 0).asFloat() / 100.0f * MAX_BRAKE_PRESSURE;
        brakeMsg.brake_stepper_engaged = true;
		brakePub.publish(brakeMsg);
		
        teensyMsg.burn = receiveValue.get("burn", false).asBool();
        //msg.light = receiveValue.get("light", false).asBool();
        //msg.blink_left = receiveValue.get("blinkL", false).asBool();
        //msg.blink_right = receiveValue.get("blinkR", false).asBool();
        teensyMsg.horn = receiveValue.get("horn", false).asBool();
        teensyPub.publish(teensyMsg);
        lastMessageReceivedTime = ros::Time::now();

        // Send message
        if(!sendValue.empty()) {
            std::string sendString = writer.write(sendValue);
            ROS_DEBUG("Send: %s", sendString.c_str());
            returnBytes = write(newsockfd, sendString.c_str(), sendString.length());
            if(returnBytes < 0) {
                ROS_ERROR("ERROR writing to socket");
                // Connection probably lost. Reopen server.
                closeRemote();
                bool success = connectRemote();
                if(!success) {
                    // TODO Handle (try reconnect continuously?)
                    ROS_ERROR("Remote connection unsuccesful");
                    std::exit(EXIT_SUCCESS);
                }
            }
        }
        ROS_DEBUG("OK");
        state = STATE_OK;
    }
    // Close connection
	closeRemote();
	ROS_INFO("Server closed.\n");
    // Exit
    std::exit(EXIT_SUCCESS);
}

bool connectRemote() {
    state = STATE_WAIT;
    // Set up server
	// TODO if using ppp0, replace wlan0 with ppp0 (upnp only neede when using router)
    //system("upnpc -a `ifconfig wlan0 | grep \"inet addr\" | cut -d : -f 2 | cut -d \" \" -f 1` 57994 57994 TCP");
	socklen_t clilen;
	struct sockaddr_in serv_addr, cli_addr;
	ROS_INFO("Server ready...\n");
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd < 0) {
        ROS_ERROR("ERROR opening socket");
        return false;
    }
	bzero((char *) &serv_addr, sizeof(serv_addr));
	int portno = REMOTE_PORT_NUMBER;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);
	while(bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		ROS_ERROR("Remote: ERROR on binding");
        usleep(1000000);    // Wait 1 s
	}
	listen(sockfd, 5);
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	if(newsockfd < 0) {
        ROS_ERROR("ERROR on accept");
        return false;
    }
	fd = newsockfd;
	ROS_INFO("Client connected...\n"); 
    state = STATE_OK;
    return true;
}

void closeRemote() {
    close(newsockfd);
	close(sockfd);
	shutdown(fd, SHUT_RD);
}