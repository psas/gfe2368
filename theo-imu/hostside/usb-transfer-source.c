/*
 * usb-transfer-source.c
 *
 *  Created on: Jul 14, 2012
 *      Author: theo
 */
#include <libusb-1.0/libusb.h>
#include <glib-2.0/glib.h>
#include <stdlib.h>

#include "include/usb-transfer-source.h"

//todo:error callback

//macro from IMU.c
#define MAX_PACKET_SIZE 		64


static gboolean prepare(GSource *source, gint *timeout_){
	usbTransferSource * src = (usbTransferSource*)source;
	*timeout_ = -1;
	return src->ready_for_next_xfer && !src->transfer_active;
}

static gboolean check(GSource *source){
	return FALSE;
}

static gboolean dispatch(GSource *source, GSourceFunc callback, gpointer user_data){
	int retErr = 0;
	usbTransferSource * usb_src = (usbTransferSource *)source;
	usb_src->ready_for_next_xfer = FALSE;
	usb_src->transfer_active = TRUE;

	retErr = libusb_submit_transfer(usb_src->transfer);
	if(retErr){
//		print_libusberror(retErr, "xfer_dispatch libusb_submit_transfer");
//		g_main_loop_quit(edfc_main);
	}
	return TRUE;
}

static void finalize(GSource *g_source){
	usbTransferSource * source = (usbTransferSource *) g_source;
	if(source->transfer){
		free(source->transfer->buffer);
		libusb_free_transfer(source->transfer);
	}
}

usbTransferSource * usb_transfer_source_new(libusb_device_handle *handle,
		unsigned char endpoint, libusb_transfer_cb_fn callback){

	static GSourceFuncs funcs = {prepare, check, dispatch, finalize};
	GSource * g_source = g_source_new(&funcs, sizeof(usbTransferSource));
	usbTransferSource * source = (usbTransferSource *) g_source;

	//todo: if handle is null
	source->handle = handle;
	source->transfer = NULL;
	source->ready_for_next_xfer = FALSE;
	source->transfer_active = FALSE;
	//setup read data transfer

	if(!(source->transfer = libusb_alloc_transfer(0))){
		g_source_destroy((GSource*)source);
		return NULL;
	}

	unsigned char * bulk_in_buffer = NULL;
	bulk_in_buffer = calloc(MAX_PACKET_SIZE, sizeof(unsigned char)); //todo: slice allocate?

	libusb_fill_bulk_transfer(source->transfer,
							  source->handle,
							  endpoint,
							  bulk_in_buffer,
							  MAX_PACKET_SIZE,
							  callback,
							  source, //creates a pointer loop. Is this bad?
							  5000);

	return source;
}

void usb_source_set_error_callback(){
	//todo:
}







