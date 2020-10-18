
#include <iostream>
#include <unistd.h>

#include <libusb.h>

static void print_devs(libusb_device **devs){
	libusb_device *dev;
	int i = 0, j = 0;
	uint8_t path[8]; 

	while ((dev = devs[i++]) != NULL) {
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0) {
			fprintf(stderr, "failed to get device descriptor");
			return;
		}

		printf("Device %04x:%04x (bus %d, device %d)",
			desc.idVendor, desc.idProduct,
			libusb_get_bus_number(dev), libusb_get_device_address(dev));

		r = libusb_get_port_numbers(dev, path, sizeof(path));
		if (r > 0) {
			printf(" path: %d", path[0]);
			for (j = 1; j < r; j++)
				printf(".%d", path[j]);
		}
		printf("\n");

        for (unsigned int k = 0; k < desc.bNumConfigurations; k++) {
			struct libusb_config_descriptor *config;
			int ret = libusb_get_config_descriptor(dev, k, &config);
			if (LIBUSB_SUCCESS != ret) {
				printf("  Couldn't retrieve descriptors\n");
				continue;
			}

            // interface
	        printf("bNumInterfaces: %d\n", config->bNumInterfaces);
	        for (unsigned int m = 0; m < config->bNumInterfaces; m++){
                printf("Interface %d, num_altsetting: %d\n", m, config->interface[m].num_altsetting);
                for (unsigned int n = 0; n < config->interface[m].num_altsetting; n++){
                	printf("    Interface:\n");
                    printf("      bInterfaceNumber:   %d\n", config->interface[m].altsetting[n].bInterfaceNumber);
                    printf("      bAlternateSetting:  %d\n", config->interface[m].altsetting[n].bAlternateSetting);
                    printf("      bNumEndpoints:      %d\n", config->interface[m].altsetting[n].bNumEndpoints);
                    printf("      bInterfaceClass:    %d\n", config->interface[m].altsetting[n].bInterfaceClass);
                    printf("      bInterfaceSubClass: %d\n", config->interface[m].altsetting[n].bInterfaceSubClass);
                    printf("      bInterfaceProtocol: %d\n", config->interface[m].altsetting[n].bInterfaceProtocol);
                    printf("      iInterface:         %d\n", config->interface[m].altsetting[n].iInterface);
                	printf("      bInterfaceNumber:   %d\n", config->interface[m].altsetting[n].bInterfaceNumber);

                    for(unsigned int x = 0; x < config->interface[m].altsetting[n].bNumEndpoints; x++){
                        printf("         bEndpointAddress:   %d\n", config->interface[m].altsetting[n].endpoint[x].bEndpointAddress);
                    }
                }
            }

			libusb_free_config_descriptor(config);
        }

		printf("\n");
	}
}


int8_t open_device_vid_pid_int(uint16_t vendor_id, uint16_t product_id, int interface_number, libusb_device_handle **dev_handle, uint8_t *bEndPointAddress){
    libusb_device ** device_list;
    ssize_t num_devices = libusb_get_device_list(NULL, &device_list);

    libusb_device *correct_dev = NULL;
    int correct_bEndPointAddress = -1;

    libusb_device_handle *_dev_handle;


    // Loop through device list
	libusb_device *dev;
    int i = 0;
	while ((dev = device_list[i++]) != NULL) {
        // Get device descriptor
        struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0) {
			fprintf(stderr, "failed to get device descriptor");
		}

        // Check vendor and product ID
        if(vendor_id == desc.idVendor && product_id == desc.idProduct){
            correct_dev = dev;

            // Now find endpoint address of the desired interface_number
            // Go into config descriptor
            for (unsigned int k = 0; k < desc.bNumConfigurations; k++) {
                struct libusb_config_descriptor *config;
                int ret = libusb_get_config_descriptor(dev, k, &config);
                if (LIBUSB_SUCCESS != ret) {
                    printf("  Couldn't retrieve descriptors\n");
                    continue;
                }

                // Find desired interface_number
                printf("Going through %d interfaces\n", config->bNumInterfaces);
                for (unsigned int m = 0; m < config->bNumInterfaces; m++){
                    printf("Interface number: %d\n", m);
                    for (unsigned int n = 0; n < config->interface[m].num_altsetting; n++){
                        printf("    Altsetting: %d\n", n);
                        printf("    Interface number: %d\n",config->interface[m].altsetting[n].bInterfaceNumber);
                        if(config->interface[m].altsetting[n].bInterfaceNumber == interface_number){
                            // There can be multiple endpoints for one interface.. what does that represent?
                            // We guess the first is correct
                            correct_bEndPointAddress = config->interface[m].altsetting[n].endpoint[0].bEndpointAddress;
                        }

                        // Prints all endpoint addresses
                        /*
                        for(unsigned int x = 0; x < config->interface[m].altsetting[n].bNumEndpoints; x++){
                            printf("        bEndpointAddress: %d\n", config->interface[m].altsetting[n].endpoint[x].bEndpointAddress);
                        }*/
                        break; // do not loop through any more interfaces
                    }
                }
                libusb_free_config_descriptor(config);
            }
            break; // do not loop through any more devices
        }

    }

    int r = LIBUSB_ERROR_OTHER;
    if(correct_dev != NULL){
        if(correct_bEndPointAddress != -1){
            r = libusb_open(correct_dev, &_dev_handle);
            *bEndPointAddress = correct_bEndPointAddress;
            *dev_handle = _dev_handle;
            printf("libusb_open returned %d\n", r);
        } else {
            printf("Interface number not found.\n");
        }
    } else {
        printf("USB device not found.\n");
    }

    libusb_free_device_list(device_list, 1);
    return r;
}