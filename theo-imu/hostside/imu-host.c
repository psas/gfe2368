/*
 * imu-host.c
 *
 *  Created on: Jun 9, 2012
 *      Author: theo
 */

#include <stdlib.h>
#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include <glib-2.0/glib.h>
#include <time.h>
#include <poll.h>

#include "include/imu-host.h"
#include "include/libusb-gsource.h"
#include "include/usb-transfer-source.h"
#include "../include/imu-device-host-interface.h"

//todo: thread safe?
//todo: handle all the errors
//todo: better error handling
//todo: handle device reset/disconnect-reconnect

#define NUM_IFACES	1

GMainLoop * edfc_main = NULL; //todo:ugg, need better data flow

void print_libusb_error(int libusberrno, char* str) {
	fprintf(stderr, "**%s: %s, %d\n", str, libusb_error_name(libusberrno), libusberrno);
}

void print_libusb_transfer_error(int status, char* str){
	switch(status){
	case LIBUSB_TRANSFER_COMPLETED:
		fprintf(stderr, "**%s: LIBUSB_TRANSFER_COMPLETED\n", str);
		break;
	case LIBUSB_TRANSFER_ERROR:
		fprintf(stderr, "**%s: LIBUSB_TRANSFER_ERROR\n", str);
		break;
	case LIBUSB_TRANSFER_TIMED_OUT:
		fprintf(stderr, "**%s: LIBUSB_TRANSFER_TIMED_OUT\n", str);
		break;
	case LIBUSB_TRANSFER_CANCELLED:
		fprintf(stderr, "**%s: LIBUSB_TRANSFER_CANCELLED\n", str);
		break;
	case LIBUSB_TRANSFER_STALL:
		fprintf(stderr, "**%s: LIBUSB_TRANSFER_STALL\n", str);
		break;
	case LIBUSB_TRANSFER_NO_DEVICE:
		fprintf(stderr, "**%s: LIBUSB_TRANSFER_NO_DEVICE\n", str);
		break;
	case LIBUSB_TRANSFER_OVERFLOW:
		fprintf(stderr, "**%s: LIBUSB_TRANSFER_OVERFLOW\n", str);
		break;
	default:
		fprintf(stderr, "***%s: Unknown transfer status %i***\n", str, status);
		break;
	}
}

gboolean is_imu_device(libusb_device * device){
	struct libusb_device_descriptor descr;
	int retErr = libusb_get_device_descriptor(device, &descr);
	if(retErr){
		print_libusb_error(retErr, "is_imu_device libusb_get_device_descriptor");
		return FALSE;
	}

	if(descr.idVendor      == 0xFFFF && \
	   descr.idProduct     == 0x0005 && \
	   descr.bcdDevice     == 0x0100 && \
	   descr.iManufacturer == 0x01   && \
	   descr.iProduct      == 0x02   && \
	   descr.iSerialNumber == 0x03 ){
		return TRUE;
	}

	return FALSE;
}

libusb_device_handle * open_usb_device_handle(libusb_context * context,
		gboolean (*is_device)(libusb_device * device),
		int * iface_num, int num_ifaces){
	//todo: handle cleanup on errors better. Dare I try goto?
	libusb_device **list = NULL;
	libusb_device *found = NULL;
	ssize_t num_usb_dev = 0;
	ssize_t i = 0;
	libusb_device_handle *handle = NULL;

	int retErr = libusb_get_device_list(context, &list);
	if(retErr < 0){
		print_libusb_error(retErr, "open_usb_device_handle libusb_get_device_list");
		return NULL;
	}else{
		num_usb_dev = retErr;
	}

	for(i = 0; i < num_usb_dev; ++i){
		libusb_device *device = list[i];
		if(is_device(device) == TRUE){
			found = device;
			break;
		}
	}
	if(found){
		retErr = libusb_open(found, &handle);
		if(retErr){
			print_libusb_error(retErr, "open_usb_device_handle libusb_open");
			libusb_free_device_list(list, 1);
			return NULL;
		}
		for(i=0; i < num_ifaces; ++i){
			retErr = libusb_kernel_driver_active(handle, iface_num[i]);
			if(retErr < 0){
				print_libusb_error(retErr, "open_usb_device_handle libusb_kernel_driver_active");
				libusb_close(handle);
				libusb_free_device_list(list, 1);
				return NULL;
			}else if(retErr > 0){
				retErr = libusb_detach_kernel_driver(handle, iface_num[i]);
				if(retErr){
					print_libusb_error(retErr, "open_usb_device_handle libusb_detach_kernel_driver");
					libusb_close(handle);
					libusb_free_device_list(list, 1);
					return NULL;
				}
			}
			retErr = libusb_claim_interface(handle, iface_num[i]);
			if(retErr){
				print_libusb_error(retErr, "open_usb_device_handle libusb_claim_interface");
				libusb_attach_kernel_driver(handle, iface_num[i]);
				libusb_close(handle);
				libusb_free_device_list(list, 1);
				return NULL;
			}
		}
	}
	libusb_free_device_list(list, 1);
	return handle;
}

void bulk_in_cb(struct libusb_transfer *transfer){
	//user data is the address of the transfer source
	usbTransferSource * source = (usbTransferSource*)transfer->user_data;
	imuPacket pkt;
	int numPkts;
	unsigned int pktOffset;
	unsigned char *buf = transfer->buffer;

	switch(transfer->status){
	case LIBUSB_TRANSFER_COMPLETED:
		for(numPkts = transfer->actual_length/IMU_PACKET_LENGTH; numPkts > 0; --numPkts){
			pktOffset = (numPkts-1)*IMU_PACKET_LENGTH;
			fill_imu_packet(&pkt, buf+pktOffset);
			printf("%s: X: %5d, Y: %5d, Z: %5d\n",
					imu_pkt_id_to_str(&pkt),
					pkt.x/16, pkt.y/16, pkt.z/16);
		}
		source->ready_for_next_xfer = TRUE;
		break;
	case LIBUSB_TRANSFER_CANCELLED:
		//do nothing.
		break;
	default:
		print_libusb_transfer_error(transfer->status, "bulk_in_cb");
		printf("quit bulk_in\n");
		g_main_loop_quit(edfc_main);
		break;
	}
}

void bulk_out_cb(struct libusb_transfer *transfer){
	if(transfer->status != LIBUSB_TRANSFER_COMPLETED){
		print_libusb_transfer_error(transfer->status, "bulk_in_cb");
		printf("quit bulk_out\n");
		g_main_loop_quit(edfc_main);
	}
}

gboolean read_g_stdin(GIOChannel * source, GIOCondition condition, gpointer data){
	usbTransferSource ** bulkIO = (usbTransferSource **)data;
	usbTransferSource * bulk_out = bulkIO[1];
	usbTransferSource * bulk_in = bulkIO[0];
	unsigned char * in_buf = NULL;
	gsize bytes_read = 0;
	gsize terminator_pos = 0;
	GError * error = NULL;
	printf("read_g_stdin callback\n");

	if(condition & ~(G_IO_IN | G_IO_ERR | G_IO_HUP)){
		printf("**Unknown GIOCondition\n");
	}
	if(condition & G_IO_ERR){
		printf("**stdin error\n");
		g_main_loop_quit(edfc_main);
		return FALSE;
	}
	if(condition & G_IO_HUP){
		printf("**stdin hung up\n");
		g_main_loop_quit(edfc_main);
		return FALSE;
	}
	if(condition & G_IO_IN){
		g_io_channel_read_line(source, (gchar**)&in_buf, &bytes_read, &terminator_pos, &error);
		if(bytes_read > 0){ //todo: handle series of chars?
			switch(in_buf[0]){
				case 'm':
					printf("(r)-Reset, (s)-Stop, (q)-Quit\n");
					break;
				case '+':
				case '-':
					printf("input: %c\n", in_buf[0]);
					if(usb_transfer_submit(bulk_out, in_buf, 1))
						printf("last char transfer not yet submitted\n");
					break;
				case 's':
					printf("input: %c\n", in_buf[0]);
					if(usb_transfer_submit(bulk_out, in_buf, 1)){
						printf("last char transfer not yet submitted\n");
					}else{
						usb_transfer_cancel(bulk_in);
					}
					break;
				case 'r':
				case 'g':
					printf("input: %c\n", in_buf[0]);
					if(usb_transfer_submit(bulk_out, in_buf, 1)){
						printf("last char transfer not yet submitted\n");
					}else{
						usb_transfer_submit(bulk_in, NULL, MAX_PACKET_SIZE);
					}
					break;
				case 'q':
					printf("\nquit\n");
					//todo: send s to IMU?
					usb_transfer_cancel(bulk_in);
					usb_transfer_cancel(bulk_out);
					g_main_loop_quit(edfc_main);
					break;
				default:
					printf("unknown input: %c\n", in_buf[0]);
					break;
			}
			g_free(in_buf);
		}
	}
	return TRUE;
}

void libusb_mainloop_error_cb(int timeout, int handle_events, GMainLoop * loop){
	if(timeout)
		print_libusb_error(timeout, "libusb timeout");
	if(handle_events)
		print_libusb_error(handle_events, "libusb handle_events");
	printf("quit libusb\n");
	g_main_loop_quit(loop);
}

int main(){
	int usbErr = 0;
	int iface_num[NUM_IFACES];
	iface_num[0]= 1; // bulk interface
	libusb_context *imu_host = NULL;
	libusb_device_handle *imu_handle = NULL;
	libusbSource * usb_source = NULL;
	usbTransferSource * bulk_in = NULL;
	usbTransferSource * bulk_out = NULL;
	usbTransferSource * bulkIO[2];

	GMainContext * edfc_context = NULL;
	GIOChannel * g_stdin = NULL;
	GSource * gs_stdin = NULL;

	usbErr = libusb_init(&imu_host);
	if(usbErr){
		print_libusb_error(usbErr, "libusb_init");
		exit(EXIT_FAILURE);
	}
	libusb_set_debug(imu_host, 3);

	imu_handle = open_usb_device_handle(imu_host, is_imu_device, iface_num, NUM_IFACES);
	if(!imu_handle){
		printf("**imu_handle acquisition error\n");
		libusb_exit(imu_host);
		exit(EXIT_FAILURE);
	}

	usb_source = libusb_source_new(imu_host);
//	g_source_set_callback((GSource*) usb_source, (GSourceFunc)libusb_mainloop_error_cb, &edfc_main, NULL);

	bulk_in = usb_transfer_source_new(imu_handle, BULK_IN_EP, bulk_in_cb);
	bulk_out = usb_transfer_source_new(imu_handle, BULK_OUT_EP, bulk_out_cb);

	edfc_context = g_main_context_new(); //edfc == event driven flight computer
	edfc_main = g_main_loop_new(edfc_context, FALSE);
	g_stdin = g_io_channel_unix_new(fileno(stdin));
	if(!g_stdin){
		printf("error creating g_stdin\n");
	}

	g_source_attach((GSource*) usb_source, edfc_context);
	g_source_attach((GSource*) bulk_in, edfc_context);
	g_source_attach((GSource*) bulk_out, edfc_context);
	gs_stdin = g_io_create_watch(g_stdin, G_IO_IN | G_IO_ERR | G_IO_HUP);
	bulkIO[0] = bulk_in;
	bulkIO[1] = bulk_out;
	g_source_set_callback(gs_stdin, (GSourceFunc)read_g_stdin, bulkIO, NULL);
	g_source_attach(gs_stdin, edfc_context);

	printf("beginning main loop\n");
	g_main_loop_run(edfc_main);
	printf("main loop finished\n");
//cleanup
	g_source_destroy(gs_stdin);
	g_io_channel_shutdown(g_stdin, TRUE, NULL); //todo: fix null
	g_source_destroy((GSource*) bulk_out);
	g_source_destroy((GSource*) bulk_in);
	g_source_destroy((GSource*) usb_source);

	g_main_loop_unref(edfc_main);
	g_main_context_unref(edfc_context);

	usbErr = libusb_release_interface(imu_handle, iface_num[0]);
	if(usbErr) print_libusb_error(usbErr, "exit libusb_release_interface");
	usbErr = libusb_attach_kernel_driver(imu_handle, iface_num[0]);
	if(usbErr) print_libusb_error(usbErr, "exit libusb_attach_kernel_driver");
	libusb_close(imu_handle);
	libusb_exit(imu_host);

	exit(EXIT_SUCCESS);
}
