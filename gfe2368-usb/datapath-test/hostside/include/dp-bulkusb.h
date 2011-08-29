

/*
 * dp-bulkusb.h
 */

#ifndef _DP_BULKUSB_H
#define _DP_BULKUSB_H

#include <stdint.h>

#ifdef VERBOSE
    #define DBG             printf
#else
    #define DBG(x ...)
#endif
#define 	MAX_BULK_XFERSIZE			64

// PSAS devices - not official
#define 	IDVENDOR	                ((uint16_t) 0xffff)
#define		IDPRODUCT	                ((uint16_t) 0x0005)

#define	    EP_BULK_IN	                ((uint8_t ) (2 | LIBUSB_ENDPOINT_IN))
#define	    EP_BULK_OUT	                ((uint8_t ) (5 | LIBUSB_ENDPOINT_OUT))

// termios values
#define     VMIN_VALUE                  0
#define     VTIME_VALUE                 0


// Globals

static struct   libusb_transfer*        bulk_xfer_in  = NULL;
//static struct   libusb_transfer*        bulk_xfer_out = NULL;

static          uint8_t                 bulk_buf_in[MAX_BULK_XFERSIZE];
//static          uint8_t                 bulk_buf_out[MAX_BULK_XFERSIZE];

static 			int32_t	                exit_test     = 0;

static struct   libusb_device_handle*   devh          = NULL;


#endif
