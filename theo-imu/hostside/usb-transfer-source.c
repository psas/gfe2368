/*
 * usb-transfer-source.c
 *
 *  Created on: Jul 14, 2012
 *      Author: theo
 */
#include <libusb-1.0/libusb.h>
#include <glib-2.0/glib.h>
#include <stdlib.h>
#include <string.h>

#include "include/usb-transfer-source.h"
#include "../include/imu-device-host-interface.h"

//todo:error callback
//todo: does this provide significant benefit? I dont really think so >,<

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

//inception.jpg, WE NEED TO GO DEEPER
static void usb_transfer_source_callback(struct libusb_transfer *transfer){
	usbTransferSource * usb_src = (usbTransferSource *)transfer->user_data;
	usb_src->transfer_active = FALSE;
	usb_src->callback(transfer);
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
	source->callback = callback;
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
							  usb_transfer_source_callback,
							  source, //creates a pointer loop. Is this bad?
							  5000);

	return source;
}

int usb_transfer_submit(usbTransferSource * source, unsigned char * buf, int buf_len){
	int len = buf_len < MAX_PACKET_SIZE ? buf_len : MAX_PACKET_SIZE;
	if(source->transfer_active){
		return -1;
	}else{
		if(buf)
			memcpy(source->transfer->buffer, buf, len);
		source->transfer->length = len;
		source->ready_for_next_xfer = TRUE;
	}
	return 0;
}

int usb_transfer_cancel(usbTransferSource * source){
	source->ready_for_next_xfer = FALSE;
	return libusb_cancel_transfer(source->transfer);
}

void usb_transfer_source_set_error_callback(){
	//todo:
}







