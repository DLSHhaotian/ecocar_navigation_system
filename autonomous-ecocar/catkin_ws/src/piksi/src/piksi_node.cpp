#include <ros/ros.h>
#include <piksi/piksi_driver.hpp>
#include <libserialport.h> 
#include <serial_tools.h>
#include <cstdlib>
#include <piksi/remote_base_sync.h>

#define USE_APP_REMOTE_BASE_SYNC 1
#define PIKSI_REMOTE_LOOP_RATE 10

enum NodeState {
    STATE_FATAL_ERROR = -1, // For unrecoverable errors
    STATE_OK,               // When OK
    STATE_WAIT              // Node is not ready or trying to recover from an error
};

NodeState state;

int main(int argc, char *argv[]) {
	state = STATE_WAIT;
    ros::init(argc, argv, "piksi_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

	ROS_INFO("Starting Piksi Driver. Searching for Piksi...");

	char* portName = NULL;
    while(ros::ok() && portName == NULL) {
        portName = get_port_name(PIKSI_SERIAL_NUMBER);
        ros::spinOnce();
        usleep(1000);   // Wait 1 ms
        ROS_INFO_THROTTLE(5, "Waiting for Piksi connection...");
    }
	if(portName == NULL) return 0;	// This should only happen if the user exits while the node is waiting for a connection

    std::string port(portName);
    nh_priv.param("port", port, port);

	// Start Piksi driver
    swiftnav_piksi::PIKSI piksi(nh, nh_priv, portName);

    ROS_DEBUG("Opening Piksi on %s", port.c_str());
    if(!piksi.PIKSIOpen()) {
		ROS_ERROR("Failed to open Piksi on %s", port.c_str());
    	std::exit(EXIT_FAILURE);
	} else {
		ROS_INFO("Piksi opened successfully on %s", port.c_str());
		state = STATE_OK;
	}

#if USE_APP_REMOTE_BASE_SYNC
	remoteBaseSyncInitialize();
#endif
	
	// All Piksi data and messages is handled in callbacks
    // Loop
    ros::Rate rate(PIKSI_REMOTE_LOOP_RATE);
    while(ros::ok()) {
        ros::spinOnce();
#if USE_APP_REMOTE_BASE_SYNC
        remoteBaseSyncLoop();
#endif
        rate.sleep();
    }

    std::exit(EXIT_SUCCESS);
}