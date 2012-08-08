/*
 * usb-speed-test-host.c
 *
 *  Created on: Jul 24, 2012
 *      Author: theo
 */

/*
 * imu-host.c
 *
 *  Created on: Jun 9, 2012
 *      Author: theo
 */

#include <stdlib.h>
#include <stdio.h>
#include <libusb.h>
#include <glib.h>
#include <time.h>
#include <poll.h>

#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include "usb-speed-test-host.h"
#include "libusb-gsource.h"
#include "imu-device-host-interface.h"

//todo: thread safe?
//todo: handle all the errors
//todo: better error handling
//todo: handle device reset/disconnect-reconnect
//todo: printf( to fprintf(stderr,


#define NUM_IFACES	1

#define NUM_EPS					32

#define CTRL_IN_EP				0x80
#define CTRL_OUT_EP				0x00
#define INTR_IN_EP              0x81
#define INTR_OUT_EP				0x01
#define BULK_IN_EP              0x82
#define BULK_OUT_EP             0x02
#define ISOC_IN_EP				0x83
#define ISOC_OUT_EP				0x03

//see lpc23xx user manual p318
/** convert from endpoint address to endpoint index */
#define EP2IDX(bEP) ((((bEP)&0xF)<<1)|(((bEP)&0x80)>>7))
/** convert from endpoint index to endpoint address */
#define IDX2EP(idx) ((((idx)<<7)&0x80)|(((idx)>>1)&0xF)

#define CTRL_IN_IDX				EP2IDX(CTRL_IN_EP)
#define CTRL_OUT_IDX			EP2IDX(CTRL_OUT_EP)
#define INTR_IN_IDX             EP2IDX(INTR_IN_EP)
#define INTR_OUT_IDX			EP2IDX(INTR_OUT_EP)
#define BULK_IN_IDX             EP2IDX(BULK_IN_EP)
#define BULK_OUT_IDX            EP2IDX(BULK_OUT_EP)
#define ISOC_IN_IDX				EP2IDX(ISOC_IN_EP)
#define ISOC_OUT_IDX			EP2IDX(ISOC_OUT_EP)


#define MAX_PACKET_SIZE 		64

#define STOPPED  0x00
#define CTRL_REQ 0x01
#define INTR_REQ 0x02
#define BULK_REQ 0x04
#define ISOC_REQ 0x08
#define READ_DATA 0x10

struct libusb_transfer * endpoint[NUM_EPS];

GMainLoop * edfc_main = NULL; //todo:ugg, need better data flow
int sfd;

gboolean is_imu_device(libusb_device * device){
	struct libusb_device_descriptor descr;
	int retErr = libusb_get_device_descriptor(device, &descr);
	if(retErr){
		print_libusb_error(retErr,"is_imu_device libusb_get_device_descriptor");
		return FALSE;
	}

	if(descr.idVendor == 0xFFFF && descr.idProduct == 0x0005){
		//todo: more ID methods
		return TRUE;
	}

	return FALSE;
}

void common_in_cb(struct libusb_transfer *transfer){
	unsigned char *buf = transfer->buffer;

	//int retErr;
	int i;
	int bytes_written;

	switch(transfer->status){
	case LIBUSB_TRANSFER_COMPLETED:
		for(i = 0; i < transfer->actual_length; ++i){
			if(buf[i] == 'A'){
//				printf("U\n");
				bytes_written = write(sfd, "U", 1);
				if (bytes_written == -1){
					   perror("Unable to write to ttyS0");
				}
			}
		}
		libusb_submit_transfer(transfer);
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


void ctrl_in_cb(struct libusb_transfer *transfer){
	unsigned char *buf = libusb_control_transfer_get_data(transfer);
	//int retErr;
	int i;
	int bytes_written;

	switch(transfer->status){
	case LIBUSB_TRANSFER_COMPLETED:
		for(i = 0; i < transfer->actual_length; ++i){
			if(buf[i] == 'A'){
				bytes_written = write(sfd, "U", 1);
				if (bytes_written == -1){
					   perror("Unable to write to ttyS0");
				}
			}
		}
		libusb_submit_transfer(transfer);
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
void ctrl_out_cb(struct libusb_transfer *transfer){
	struct libusb_control_setup * sent = libusb_control_transfer_get_setup(transfer);
	if(transfer->status != LIBUSB_TRANSFER_COMPLETED){
		print_libusb_transfer_error(transfer->status, "bulk_out_cb");
		printf("quit bulk_out\n");
		g_main_loop_quit(edfc_main);
	}

	switch(sent->bRequest){
	case CTRL_REQ:
		libusb_fill_control_setup(endpoint[CTRL_IN_IDX]->buffer,
			LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
			READ_DATA, 0, 0, 1);
		libusb_submit_transfer(endpoint[CTRL_IN_IDX]);
		break;
	case INTR_REQ:
		libusb_submit_transfer(endpoint[INTR_IN_IDX]);
		break;
	case BULK_REQ:
		libusb_submit_transfer(endpoint[BULK_IN_IDX]);
		break;
	case ISOC_REQ:
		libusb_submit_transfer(endpoint[ISOC_IN_IDX]);
		break;
	default:
		break;
	}
}
void intr_in_cb(struct libusb_transfer *transfer){
	common_in_cb(transfer);
}
void intr_out_cb(struct libusb_transfer *transfer){
	if(transfer->status != LIBUSB_TRANSFER_COMPLETED){
		print_libusb_transfer_error(transfer->status, "bulk_out_cb");
		printf("quit bulk_out\n");
		g_main_loop_quit(edfc_main);
	}
}
void bulk_in_cb(struct libusb_transfer *transfer){
	common_in_cb(transfer);
}
void bulk_out_cb(struct libusb_transfer *transfer){
	if(transfer->status != LIBUSB_TRANSFER_COMPLETED){
		print_libusb_transfer_error(transfer->status, "bulk_out_cb");
		printf("quit bulk_out\n");
		g_main_loop_quit(edfc_main);
	}
}
void isoc_in_cb(struct libusb_transfer *transfer){
	unsigned char *buf = libusb_get_iso_packet_buffer_simple(transfer, 0);
	//int retErr;
	int i;
	int bytes_written;

	switch(transfer->status){
	case LIBUSB_TRANSFER_COMPLETED:
		for(i = 0; i < transfer->actual_length; ++i){
			if(buf[i] == 'A'){
//				printf("U\n");
				bytes_written = write(sfd, "U", 1);
				if (bytes_written == -1){
					   perror("Unable to write to ttyS0");
				}
			}
		}
		libusb_submit_transfer(transfer);
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
void isoc_out_cb(struct libusb_transfer *transfer){
	if(transfer->status != LIBUSB_TRANSFER_COMPLETED){
		print_libusb_transfer_error(transfer->status, "bulk_out_cb");
		printf("quit bulk_out\n");
		g_main_loop_quit(edfc_main);
	}
}
gboolean read_g_stdin(GIOChannel * source, GIOCondition condition, gpointer data){
	struct libusb_transfer ** endpoints = (struct libusb_transfer **)data;
	struct libusb_transfer * ctrl_out = endpoints[CTRL_OUT_IDX];
	unsigned char * stdin_buf = NULL;
	gsize bytes_read = 0;
	gsize terminator_pos = 0;
	GError * error = NULL;
	int i;

	if(condition & ~(G_IO_IN | G_IO_ERR | G_IO_HUP)){
		printf("**Unknown GIOCondition\n");
		g_main_loop_quit(edfc_main);
		return FALSE;
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
	//then condition must be G_IO_IN
	g_io_channel_read_line(source, (gchar**)&stdin_buf, &bytes_read, &terminator_pos, &error);
	if(bytes_read > 0){ //todo: handle series of chars?
		printf("input: %c\n", stdin_buf[0]);
		switch(stdin_buf[0]){
		case 'c':
			libusb_fill_control_setup(ctrl_out->buffer,
					LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
					CTRL_REQ, 0, 0, 0);
			libusb_submit_transfer(ctrl_out);
			break;
		case 'i':
			libusb_fill_control_setup(ctrl_out->buffer,
					LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
					INTR_REQ, 0, 0, 0);
			libusb_submit_transfer(ctrl_out);
			break;
		case 'b':
			libusb_fill_control_setup(ctrl_out->buffer,
					LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
					BULK_REQ, 0, 0, 0);
			libusb_submit_transfer(ctrl_out);
			break;
		case 's':
			libusb_fill_control_setup(ctrl_out->buffer,
					LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
					ISOC_REQ, 0, 0, 0);
			libusb_submit_transfer(ctrl_out);
			break;
		case 'p':
			libusb_fill_control_setup(ctrl_out->buffer,
					LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
					STOPPED, 0, 0, 0);
			libusb_submit_transfer(ctrl_out);

			for(i = 0; i < NUM_EPS; ++i){
				if(endpoint[i]){
					libusb_cancel_transfer(endpoint[i]);
				}
			}
			printf("\npaused\n");
			break;
		case 'q':
			libusb_fill_control_setup(ctrl_out->buffer,
					LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
					STOPPED, 0, 0, 0);
			libusb_submit_transfer(ctrl_out);

			for(i = 0; i < NUM_EPS; ++i){
				if(endpoint[i]){
					libusb_cancel_transfer(endpoint[i]);
				}
			}
			printf("\nquit\n");
			g_main_loop_quit(edfc_main);
			break;
		default:
			printf("unknown input\n");
			break;
		}
		g_free(stdin_buf);
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

int open_port(void){
   int fd; /* File descriptor for the port */

   fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
   if (fd == -1){
	   perror("open_port: Unable to open /dev/ttyS0");
  }
  return (fd);
}

struct libusb_transfer * allocate_default_transfer(libusb_device_handle * handle,
		struct libusb_endpoint_descriptor * ep, libusb_transfer_cb_fn callback)
{
	unsigned char * buf = malloc(ep->wMaxPacketSize);;
	struct libusb_transfer * transfer = libusb_alloc_transfer(0);

	transfer->dev_handle = handle;
	transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;
	transfer->endpoint = ep->bEndpointAddress;
	transfer->type = ep->bmAttributes & 0x03;

	switch(ep->bmAttributes & 0x03){
	case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
	case LIBUSB_TRANSFER_TYPE_INTERRUPT:
	case LIBUSB_TRANSFER_TYPE_BULK:

		break;
	default:
		break;
	}
	transfer->timeout = 0; //todo: interrupt transfers?
	transfer->length = ep->wMaxPacketSize;
	transfer->callback = callback;
	transfer->user_data = NULL;
	transfer->buffer = buf;

	return transfer;
}

int main(){
	char in;
	int usbErr = 0;
	int iface_nums[NUM_IFACES];
	iface_nums[0]= 0; // test interface
	libusb_context *imu_host = NULL;
	libusb_device_handle *imu_handle = NULL;
	libusbSource * usb_source = NULL;
	//ssize_t bytes_written = 0;
	struct termios attrib;

	unsigned char * ctrl_in_buffer = NULL;
	unsigned char * ctrl_out_buffer = NULL;
	unsigned char * intr_in_buffer = NULL;
	unsigned char * intr_out_buffer = NULL;
	unsigned char * bulk_in_buffer = NULL;
	unsigned char * bulk_out_buffer = NULL;
	unsigned char * isoc_in_buffer = NULL;
	unsigned char * isoc_out_buffer = NULL;


	GMainContext * edfc_context = NULL;
	GIOChannel * g_stdin = NULL;
	GSource * gs_stdin = NULL;

	sfd = open_port();
	tcgetattr(sfd, &attrib);
	cfsetispeed(&attrib, B115200);
	tcsetattr(sfd, TCSANOW, &attrib);

	//initialize libusb
	usbErr = libusb_init(&imu_host);
	if(usbErr){
		print_libusb_error(usbErr, "libusb_init");
		exit(EXIT_FAILURE);
	}
	libusb_set_debug(imu_host, 3);
	usb_source = libusb_source_new(imu_host);
	g_source_set_callback((GSource*) usb_source, (GSourceFunc)libusb_mainloop_error_cb, &edfc_main, NULL);

	//get the usb device
	do{
		imu_handle = open_usb_device_handle(imu_host, is_imu_device, iface_nums, NUM_IFACES);
		if(!imu_handle){
			printf("**imu_handle acquisition error, retry (y/n)?\n");
			in = getchar();
			while(getchar() != '\n');
			if(in != 'y'){
				printf("quitting\n");
				libusb_exit(imu_host);
				exit(EXIT_FAILURE);
			}
		}
	}while(!imu_handle);

	//allocate and default fill transfers
	endpoint[CTRL_IN_IDX]  = libusb_alloc_transfer(0);
	endpoint[CTRL_OUT_IDX] = libusb_alloc_transfer(0);
	endpoint[INTR_IN_IDX]  = libusb_alloc_transfer(0);
	endpoint[INTR_OUT_IDX] = libusb_alloc_transfer(0);
	endpoint[BULK_IN_IDX]  = libusb_alloc_transfer(0);
	endpoint[BULK_OUT_IDX] = libusb_alloc_transfer(0);
	endpoint[ISOC_IN_IDX]  = libusb_alloc_transfer(1);
	endpoint[ISOC_OUT_IDX] = libusb_alloc_transfer(1);
	//todo: slice allocate?
	ctrl_in_buffer  = calloc(MAX_PACKET_SIZE + LIBUSB_CONTROL_SETUP_SIZE, sizeof(unsigned char));
	ctrl_out_buffer = calloc(MAX_PACKET_SIZE + LIBUSB_CONTROL_SETUP_SIZE, sizeof(unsigned char));
	intr_in_buffer  = calloc(MAX_PACKET_SIZE, sizeof(unsigned char));
	intr_out_buffer = calloc(MAX_PACKET_SIZE, sizeof(unsigned char));
	bulk_in_buffer  = calloc(MAX_PACKET_SIZE, sizeof(unsigned char));
	bulk_out_buffer = calloc(MAX_PACKET_SIZE, sizeof(unsigned char));
	isoc_in_buffer  = calloc(MAX_PACKET_SIZE, sizeof(unsigned char));
	isoc_out_buffer = calloc(MAX_PACKET_SIZE, sizeof(unsigned char));

	libusb_fill_control_transfer(endpoint[CTRL_IN_IDX],
								 imu_handle,
								 ctrl_in_buffer,
								 ctrl_in_cb,
								 NULL,
								 0);
	libusb_fill_control_transfer(endpoint[CTRL_OUT_IDX],
								 imu_handle,
								 ctrl_out_buffer,
								 ctrl_out_cb,
								 NULL,
								 0);

	libusb_fill_interrupt_transfer(endpoint[INTR_IN_IDX],
								   imu_handle,
								   INTR_IN_EP,
								   intr_in_buffer,
								   MAX_PACKET_SIZE,
								   intr_in_cb,
								   NULL,
								   0);
	libusb_fill_interrupt_transfer(endpoint[INTR_OUT_IDX],
								   imu_handle,
								   INTR_OUT_EP,
								   intr_out_buffer,
								   1,
								   intr_out_cb,
								   NULL,
								   0);

	libusb_fill_bulk_transfer(endpoint[BULK_IN_IDX],
							  imu_handle,
							  BULK_IN_EP,
							  bulk_in_buffer,
							  MAX_PACKET_SIZE,
							  bulk_in_cb,
							  NULL,
							  0);
	libusb_fill_bulk_transfer(endpoint[BULK_OUT_IDX],
							  imu_handle,
							  BULK_OUT_EP,
							  bulk_out_buffer,
							  1,
							  bulk_out_cb,
							  NULL,
							  0);

	libusb_fill_iso_transfer(endpoint[ISOC_IN_IDX],
							 imu_handle,
							 ISOC_IN_EP,
							 isoc_in_buffer,
							 MAX_PACKET_SIZE,
							 1,
							 isoc_in_cb,
							 NULL,
							 0);
	libusb_set_iso_packet_lengths(endpoint[ISOC_IN_IDX], 1);
	libusb_fill_iso_transfer(endpoint[ISOC_OUT_IDX],
							 imu_handle,
							 ISOC_OUT_EP,
							 isoc_out_buffer,
							 1,
							 1,
							 isoc_out_cb,
							 NULL,
							 0);

	edfc_context = g_main_context_new(); //edfc == event driven flight computer
	edfc_main = g_main_loop_new(edfc_context, FALSE);

	g_stdin = g_io_channel_unix_new(fileno(stdin));
	if(!g_stdin){
		printf("error creating g_stdin\n");
	}
	g_source_attach((GSource*) usb_source, edfc_context);
	gs_stdin = g_io_create_watch(g_stdin, G_IO_IN | G_IO_ERR | G_IO_HUP);
	g_source_set_callback(gs_stdin, (GSourceFunc)read_g_stdin, endpoint, NULL);
	g_source_attach(gs_stdin, edfc_context);

	printf("beginning main loop\n");
	g_main_loop_run(edfc_main);
	printf("main loop finished\n");
//cleanup
	g_source_destroy(gs_stdin);
	g_io_channel_shutdown(g_stdin, TRUE, NULL); //todo: fix null
	free(ctrl_in_buffer);
	free(ctrl_out_buffer);
	free(intr_in_buffer);
	free(intr_out_buffer);
	free(bulk_in_buffer);
	free(bulk_out_buffer);
	free(isoc_in_buffer);
	free(isoc_out_buffer);
	libusb_free_transfer(endpoint[CTRL_IN_IDX]);
	libusb_free_transfer(endpoint[CTRL_OUT_IDX]);
	libusb_free_transfer(endpoint[INTR_IN_IDX]);
	libusb_free_transfer(endpoint[INTR_OUT_IDX]);
	libusb_free_transfer(endpoint[BULK_IN_IDX]);
	libusb_free_transfer(endpoint[BULK_OUT_IDX]);
	libusb_free_transfer(endpoint[ISOC_IN_IDX]);
	libusb_free_transfer(endpoint[ISOC_OUT_IDX]);
	g_source_destroy((GSource*) usb_source);

	g_main_loop_unref(edfc_main);
	g_main_context_unref(edfc_context);

	//todo: release all interfaces
	usbErr = libusb_release_interface(imu_handle, iface_nums[0]);
	if(usbErr) print_libusb_error(usbErr, "exit libusb_release_interface");
	usbErr = libusb_attach_kernel_driver(imu_handle, iface_nums[0]);
	if(usbErr) print_libusb_error(usbErr, "exit libusb_attach_kernel_driver");
	libusb_close(imu_handle);
	 //todo: deref device
	libusb_exit(imu_host);

	exit(EXIT_SUCCESS);
}

