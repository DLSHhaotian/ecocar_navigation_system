#include "JrkG2.h"

JrkG2::JrkG2(){
    
}

// Destructor should close serial connection..
JrkG2::~JrkG2(){
    disconnect();
}

void JrkG2::disconnect(){
    printf("Disconnecting from Jrk G2 USB..\n");

    int r = libusb_release_interface(dev_handle, INTERFACE_NUMBER);
    if(r != LIBUSB_SUCCESS) printf("%s\n", libusb_error_name(r));
    else printf("Interface released\n");
    
    libusb_close(dev_handle);
    libusb_exit(NULL);
}

void JrkG2::connect() {
    // Set up serial connection
    printf("Connecting to Jrk G2 USB..\n");
    libusb_init(NULL); // This function must be called before calling any other libusb function.
    libusb_set_debug(NULL, 3); // log level. 5 = debug, 3 = warn
    
    uint8_t bEndPointAddress;

    int r = open_device_vid_pid_int(VENDOR_ID, PRODUCT_ID, INTERFACE_NUMBER, &dev_handle, &bEndPointAddress);
    while(r != LIBUSB_SUCCESS){
        if(r != LIBUSB_ERROR_OTHER) printf("%s\n", libusb_error_name(r)); // other means not found, already prints an error.
        usleep(1000*100);
        r = open_device_vid_pid_int(VENDOR_ID, PRODUCT_ID, INTERFACE_NUMBER, &dev_handle, &bEndPointAddress);
    }
    printf("Connected, endpoint address: %d\n", bEndPointAddress);

    r = libusb_kernel_driver_active(dev_handle, INTERFACE_NUMBER); 
    printf("kernel driver active: %d\n", r);
    if(r == 1){
        printf("Trying to detach kernel driver..\n");
        r = libusb_detach_kernel_driver(dev_handle, INTERFACE_NUMBER);
        if(r != LIBUSB_SUCCESS) printf("%s\n", libusb_error_name(r));
    }
    r = libusb_claim_interface(dev_handle, INTERFACE_NUMBER);
    if(r != 0) printf("%s\n", libusb_error_name(r));

    endpoint = bEndPointAddress | LIBUSB_ENDPOINT_OUT;
}

bool JrkG2::setTarget(double pos){
    // Check if negative pos
    if(pos < 0) pos = 0;
    // PID parameters were tuned for target given as voltage on DAC
    // Changed to use controller USB interface
    uint16_t target = pos*4095/5; // Could be removed if PID parameters are scaled
    std::cout << "Setting target:" << target << std::endl;

    if (target > 4095) target = 4095;
    
    int length = 2;
    uint8_t data[length];
    data[0] = 0xC0 + (target & 0x1F);
    data[1] = (target >> 5) & 0x7F;
    
    int transferred;

    int r = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, SEND_TIMEOUT);
    if(r != LIBUSB_SUCCESS){
        printf("%s\n", libusb_error_name(r));
        return false;
    } 
    else printf("%d bytes of %d written\n", transferred, length);

    return true;
}
