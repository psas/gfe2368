

/*
 * dp-bulkusb.h
 */

#ifndef _DP_BULKUSB_H
#define _DP_BULKUSB_H

#ifdef VERBOSE
    #define DBG             printf
#else
    #define DBG(x ...)
#endif

// Globals
static struct libusb_device_handle *devh = NULL;

//static struct libusb_transfer *img_transfer = NULL;

#endif
