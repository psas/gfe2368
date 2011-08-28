
/*
 * dp-bulkusb.c
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <time.h>

#include "libusb-1.0/libusb.h"

#include "parse-args.h"
#include "dp-bulkusb.h"


/** \return handle to first gfe device.
 */
static int find_gfe_device(void) {
    uint16_t idvendor  = 0xffff;
    uint16_t idproduct = 0x0005;

    devh = libusb_open_device_with_vid_pid(NULL, idvendor, idproduct);

    return devh ? 0 : -EIO;
}




/**
 * \brief clean up the interface before exit
 */
void clean_interface() {
            libusb_release_interface(devh, 0);
            libusb_close(devh);
            libusb_exit(NULL);
}

/**
 * dp_task
 * \brief main bulk usb task to read and write data
 */
void dp_task() {

    int32_t ret    = 1;

    fprintf(stderr,"dp_task: not implemented. Exiting.\n");
    exit(EXIT_FAILURE);

    ret = libusb_init(NULL);
    if (ret < 0) {
    	fprintf(stderr, "dp_task: failed to init libusb..exiting\n");
    	exit(EXIT_FAILURE);
    }

    ret = find_gfe_device();
    if(ret < 0) {
    	fprintf(stderr, "dp_task: Failed to find gfe device.");
    }

    ret = libusb_claim_interface(devh, 0);
    if (ret < 0) {
    	fprintf(stderr, "dp_task: claim_interface error: %d\n", ret);
    	clean_interface();
    	exit(EXIT_FAILURE);
    }
    DBG("dp_task: claimed interface\n");
//    ret = alloc_transfers();
//    	if (r < 0)
//    		goto out_deinit;
}




int main(int argc, char* argv[]) {

	printf("**Starting %s\n", argv[0]);

	dp_task();

	printf("\n** %s Done. **\n\n", argv[0]);

	clean_interface();
	return(0);

}


