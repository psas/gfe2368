/*
 * usb-transfer-source.h
 *
 *  Created on: Jul 15, 2012
 *      Author: theo
 */

#ifndef USB_TRANSFER_SOURCE_H_
#define USB_TRANSFER_SOURCE_H_

//todo: rethink source operation - should dispatch send the request?

typedef struct usbTransferSource {
	GSource source;
	gboolean transfer_active;
	gboolean ready_for_next_xfer;
	libusb_device_handle *handle;
	struct libusb_transfer* transfer;
} usbTransferSource;

usbTransferSource * usb_transfer_source_new(libusb_device_handle *handle,
		unsigned char endpoint, libusb_transfer_cb_fn callback);

void usb_source_set_error_callback();

#endif /* USB_TRANSFER_SOURCE_H_ */
