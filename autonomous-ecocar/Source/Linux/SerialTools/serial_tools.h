//
// Created by dynamo on 5/1/17.
//

#ifndef SERIAL_TOOLS_H
#define SERIAL_TOOLS_H

#define TEENSY_SERIAL_NUMBER    "4407850"
#define GYRO_SERIAL_NUMBER	    "A700L690"
#define PIKSI_SERIAL_NUMBER	    "A904RS9J"
#define SF40_SERIAL_NUMBER	    "FT9LDIZ3"
#define PIKSI_RTK_BASE_SERIAL_NUMBER	"A904PUFP"
#define PIKSI_RTK_BASE_2_SERIAL_NUMBER	"A105ZBUC"
#define JRKG2_SERIAL_NUMBER "00223690"

#include <ros/ros.h>
#include <libserialport.h>

char* get_port_name(const char* serialNumber) {
    int i;
    struct sp_port **ports;

    sp_return error = sp_list_ports(&ports);
    if(error == SP_OK) {
        for(i = 0; ports[i]; i++) {
            char* portName = sp_get_port_name(ports[i]);
            char* usbSerialNumber = sp_get_port_usb_serial(ports[i]);
            if(usbSerialNumber != NULL && strcmp(usbSerialNumber, serialNumber) == 0) {
                ROS_INFO("Found device %s on %s", serialNumber, portName);
                return portName;
            }
        }
        sp_free_port_list(ports);
    } else {
        ROS_ERROR("Could not find device: %s", serialNumber);
        return NULL;
    }
}

#endif //SERIAL_TOOLS_H
