/*
 * libusb-gsource.h
 *
 *  Created on: Jul 14, 2012
 *      Author: theo
 */

#ifndef LIBUSB_GSOURCE_H_
#define LIBUSB_GSOURCE_H_

typedef void (*libusbSourceErrorCallback)(int, int, GMainLoop *);

typedef struct libusbSource {
	GSource source;
	GSList * fds;
	int timeout_error;
	int handle_events_error;
	libusb_context * context;
	//put gmainloop in libusbSource?
} libusbSource;

libusbSource * libusb_source_new(libusb_context * context);

#endif /* LIBUSB_GSOURCE_H_ */
