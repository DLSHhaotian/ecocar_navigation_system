#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <libserialport.h> 
#include <serial_tools.h>

#define BAUDRATE 115200
#define PIKSI_BASE_REMOTE_PORT_NUMBER 57995
#define MAX_RX_MESSAGE_SIZE 256
#define REMOTE_TIMEOUT 5        // [s]
#define SEND_TIMEOUT 50         // [ms]

ros::Time lastMessageReceivedTime;

enum NodeState {
    STATE_FATAL_ERROR = -1, // For unrecoverable errors
    STATE_OK,               // When OK
    STATE_WAIT              // Node is not ready or trying to recover from an error
};

NodeState remoteBaseSyncState;

bool connectRemoteBase();
void closeRemoteBase();

struct sp_port *port;
int fd, sockfd, newsockfd;
uint8_t dataBuffer[MAX_RX_MESSAGE_SIZE];

void connectSerial() {
    remoteBaseSyncState = STATE_WAIT;
    // Set up serial connection
    char* portName = NULL;
    while(ros::ok() && portName == NULL) {
        portName = get_port_name(PIKSI_RTK_BASE_SERIAL_NUMBER);		// A904PUFP
		if(portName == NULL) {
			portName = get_port_name(PIKSI_RTK_BASE_2_SERIAL_NUMBER);	// A105ZBUC
		}
        ros::spinOnce();
        usleep(1000);   // Wait 1 ms
        ROS_INFO_THROTTLE(5, "Waiting for Piksi RTK Port connection...");
    }
    ROS_INFO("Opening Piksi RTK Port on port '%s' \n", portName);
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
    remoteBaseSyncState = STATE_OK;
}

bool remoteBaseSyncInitialize() {
    remoteBaseSyncState = STATE_WAIT;
	ROS_INFO("Starting Piksi Remote Base Sync");

    system("~/Scripts/register_ip.sh");

	connectSerial();
	
    // Open server
    bool success = connectRemoteBase();
    if(!success) {
        // TODO Handle
        ROS_ERROR("Remote connection unsuccesful");
    }
    return success;
}

bool remoteBaseSyncLoop() {
    // Timeout
    if(ros::Time::now() - lastMessageReceivedTime > ros::Duration(REMOTE_TIMEOUT)) {
        ROS_ERROR("Piksi Remote Base last message received timeout");
    }
    // Read message
    bzero(dataBuffer, MAX_RX_MESSAGE_SIZE);
    int returnBytes = read(newsockfd, dataBuffer, MAX_RX_MESSAGE_SIZE - 1);
    if(returnBytes < 0) {
        ROS_ERROR("Piksi Remote Base ERROR reading from socket");
        // Connection probably lost. Reopen server.
        closeRemoteBase();
        bool success = connectRemoteBase();
        if(!success) {
            // TODO Handle
            ROS_ERROR("Piksi Remote Base connection unsuccesful");
            remoteBaseSyncState = STATE_FATAL_ERROR;
        }
    } else if(returnBytes == 0) {
        ROS_INFO("Piksi Remote Base Nothing received");
    }
    ROS_INFO("Piksi Remote Base message received");
    // Forward message to serial port
    sp_return spReturn = sp_blocking_write(port, dataBuffer, returnBytes, SEND_TIMEOUT);
    if(spReturn < 0) {
        ROS_ERROR("Serial error %d during write: %s", spReturn, sp_last_error_message());
        connectSerial();
    }
    remoteBaseSyncState = STATE_OK;
}

bool connectRemoteBase() {
    remoteBaseSyncState = STATE_WAIT;
    // Set up server
    //system("upnpc -a `ifconfig wlan0 | grep \"inet addr\" | cut -d : -f 2 | cut -d \" \" -f 1` 57995 57995 TCP");
	socklen_t clilen;
	struct sockaddr_in serv_addr, cli_addr;
	ROS_INFO("Server ready...\n");
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd < 0) {
        ROS_ERROR("ERROR opening socket");
        return false;
    }
	bzero((char *) &serv_addr, sizeof(serv_addr));
	int portno = PIKSI_BASE_REMOTE_PORT_NUMBER;
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
    remoteBaseSyncState = STATE_OK;
    return true;
}

void closeRemoteBase() {
    close(newsockfd);
	close(sockfd);
	shutdown(fd, SHUT_RD);
}