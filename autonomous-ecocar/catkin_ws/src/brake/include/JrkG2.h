#ifndef JRK_G2_H
#define JRK_G2_H

#include <libusb.h>

#include "libusb_tools.h"

//#define BAUDRATE 9600
#define SEND_TIMEOUT 1000

#define VENDOR_ID 0x1ffb
#define PRODUCT_ID 0x00b7
#define INTERFACE_NUMBER 2

class JrkG2{
    public:
        JrkG2();
        ~JrkG2();
        void connect();
        void disconnect();
        bool setTarget(double pos);

    private:
        struct libusb_device_handle *dev_handle; // struct?
        unsigned char endpoint;
};

#endif