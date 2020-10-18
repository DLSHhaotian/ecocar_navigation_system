#include <iostream>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include <cstdlib>
#include <libserialport.h>
//#include "serial_tools.h"

#define GYRO_SERIAL_NUMBER "A700L690"
#define TEENSY_SERIAL_NUMBER "2853910"
#define BAUDRATE 115200
#define PACKET_SIZE 8

struct sp_port *port;

void list_ports_with_all_info() {
    int i;
    struct sp_port **ports;

    sp_return error = sp_list_ports(&ports);
    if (error == SP_OK) {
        for (i = 0; ports[i]; i++) {
            printf("Port name: '%s'\n", sp_get_port_name(ports[i]));
            printf("Port description: %s\n", sp_get_port_description(ports[i]));
            int usb_bus, usb_address;
            sp_get_port_usb_bus_address(ports[i], &usb_bus, &usb_address);
            printf("USB bus: '%d' USB address: '%d'\n", usb_bus, usb_address);
            int usb_vid, usb_pid;
            sp_get_port_usb_vid_pid(ports[i], &usb_vid, &usb_pid);
            printf("USB vid: '%d' USB pid: '%d'\n", usb_vid, usb_pid);
            printf("USB manufacturer: %s\n", sp_get_port_usb_manufacturer(ports[i]));
            printf("USB product: %s\n", sp_get_port_usb_product(ports[i]));
            printf("USB serial: %s\n", sp_get_port_usb_serial(ports[i]));
        }
        sp_free_port_list(ports);
    } else {
        printf("No serial devices detected\n");
    }
    printf("\n");
}

inline bool parse_serial() {
    /*for (int i = 0; i < byte_num; i++) {
        printf("%c", byte_buff[i]);
    }
    printf("\n");*/
    short header;
    short rate_int;
    short angle_int;
    float rate_float;
    float angle_float;
    short check_sum;
    unsigned char byte_buff[PACKET_SIZE];

    if(PACKET_SIZE != sp_nonblocking_read(port, byte_buff, 8))
        return false;

    // Verify data packet header
    memcpy(&header,byte_buff,sizeof(short));
    if(header != (short)0xFFFF)
    {
        std::cout << "Header error !!!\n";
        return false;
    }

    // Copy values from data string
    memcpy(&rate_int,byte_buff+2,sizeof(short));
    memcpy(&angle_int,byte_buff+4,sizeof(short));
    memcpy(&check_sum,byte_buff+6,sizeof(short));

    // Verify checksum
    if(check_sum != (short)(0xFFFF + rate_int + angle_int))
    {
        std::cout<< "Checksum error!!\n";
        return false;
    }

    // Apply scale factors
    rate_float = rate_int/100.0;
    angle_float = angle_int/100.0;

    std::cout << "rate_float:" << rate_float << " [deg/sec]\t angle_float:" << angle_float << " [deg]\n";
    return true;
}

int main() {
    list_ports_with_all_info();

    /*
    char* portName = get_port_name(GYRO_SERIAL_NUMBER);

    printf("Opening port '%s' \n", portName);
    sp_return error = sp_get_port_by_name(portName, &port);
    if (error == SP_OK) {
        error = sp_open(port, SP_MODE_READ);
        if (error == SP_OK) {
            sp_set_baudrate(port, BAUDRATE);
            while(true) {
                //unsigned char byte_buff[8];
                //int byte_num = 0;
                //byte_num = sp_blocking_read(port, byte_buff, 8, 500);
                //bool success = parse_serial(byte_buff, byte_num);
                parse_serial();
            }

            sp_close(port);
        } else {
            printf("Error opening serial device\n");
        }
    } else {
        printf("Error finding serial device\n");
    }*/


    return 0;
}