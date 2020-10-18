
#include <iostream>
#include <unistd.h>

#include <libusb.h>

#include "libusb_tools.h"

int main(){
    uint16_t vendor_id = 0x1ffb;
    uint16_t product_id = 0x00b7;
    // I think interface 1 is correct..
    int interface_number = 2; // if the interface has no endpoints, we get segmentation fault. (0 does)

    libusb_init(NULL); // This function must be called before calling any other libusb function.
    libusb_set_debug(NULL, 5);
    
    uint8_t bEndPointAddress;

    libusb_device_handle *dev_handle;

    int r = open_device_vid_pid_int(vendor_id, product_id, interface_number, &dev_handle, &bEndPointAddress);
    while(r != LIBUSB_SUCCESS){
        printf("%s\n", libusb_error_name(r));
        usleep(1000*100);
        r = open_device_vid_pid_int(vendor_id, product_id, interface_number, &dev_handle, &bEndPointAddress);
    }
    printf("Connected, endpoint address: %d\n", bEndPointAddress);

    r = libusb_kernel_driver_active(dev_handle, interface_number); //interface_number
    printf("kernel driver active: %d\n", r);
    if(r == 1){
        printf("Trying to detach kernel driver..\n");
        r = libusb_detach_kernel_driver(dev_handle, interface_number);
        if(r != LIBUSB_SUCCESS) printf("%s\n", libusb_error_name(r));
    }
    r = libusb_claim_interface(dev_handle, interface_number);
    if(r != 0) printf("%s\n", libusb_error_name(r));

    unsigned char endpoint = bEndPointAddress | LIBUSB_ENDPOINT_OUT;

    uint16_t target = 400; 

    if (target > 4095) { target = 4095; }

    int length = 2;
    uint8_t data[length];
    data[0] = 0xC0 + (target & 0x1F);
    data[1] = (target >> 5) & 0x7F;
    
    int transferred;

    r = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, 2000);
    if(r != LIBUSB_SUCCESS) printf("%s\n", libusb_error_name(r));
    else printf("%d bytes of %d written\n", transferred, length);

    r = libusb_release_interface(dev_handle, interface_number);
    if(r != LIBUSB_SUCCESS) printf("%s\n", libusb_error_name(r));
    else printf("Interface released\n");
    
    libusb_close(dev_handle);

    libusb_exit(NULL);

}