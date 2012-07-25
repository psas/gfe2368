/*
 * libusb-gsource.c
 *
 *  Created on: Jul 14, 2012
 *      Author: theo
 */

#include <libusb-1.0/libusb.h>
#include <glib-2.0/glib.h>
#include <time.h>
#include <poll.h>
#include <stdlib.h>

#include "libusb-gsource.h"

//todo: document
//todo: are there error types I can handle here?
//todo: multithread libusb_handle_events_timeout_completed

static gboolean prepare(GSource *g_source, gint *timeout_)
{
	libusbSource *usb_src = (libusbSource *)g_source;
	struct timeval timeout;
    int retval = libusb_get_next_timeout(usb_src->context, &timeout);
    switch(retval){
		case 0:
			*timeout_ = -1;
			return FALSE;
		case 1:
			*timeout_  =  timeout.tv_sec*1000 + timeout.tv_usec/1000; //wait at most timeout, rounded down to nearest msec
			return *timeout_ == 0 ? TRUE : FALSE;
		default:
			usb_src->timeout_error = retval;
			return TRUE;
	}
    return FALSE;
}

static gboolean alt_prepare(GSource *g_source, gint *timeout_){
	*timeout_ = -1;
	return FALSE;
}

static gboolean check(GSource *g_source)
{
	libusbSource *usb_src = (libusbSource *)g_source;
	GSList * elem = usb_src->fds;
	GPollFD * g_usb_fd = NULL;

	if(!elem)
		return FALSE;

	do{
		g_usb_fd = elem->data;
		if(g_usb_fd->revents)
			return TRUE;
	}while((elem = g_slist_next(elem)));

	return FALSE;
}

static gboolean dispatch(GSource *g_source, GSourceFunc callback, gpointer user_data)
{
	// from some random bit of code on the interwebs:
	// If Dispatch returns FALSE, GLib will destroy the source.
	// src.chromium.org/svn/trunk/src/base/message_pump_glib.cc

	libusbSource *usb_src = (libusbSource *)g_source;
	libusbSourceErrorCallback errCB = (libusbSourceErrorCallback)callback;
	GMainLoop *loop = (GMainLoop*)user_data; //can I verify that this is a gmainloop?
	struct timeval nonblocking = {
			.tv_sec = 0,
			.tv_usec = 0,
	};

	if(usb_src->timeout_error && errCB != NULL){
		errCB(usb_src->timeout_error, 0, loop);
		return TRUE;
	}

	usb_src->handle_events_error = libusb_handle_events_timeout_completed(usb_src->context, &nonblocking, NULL);
	if(usb_src->handle_events_error && errCB != NULL)
		errCB(0, usb_src->handle_events_error, loop);
    return TRUE;
}

static void finalize(GSource *g_source){
	libusbSource * usb_src = (libusbSource*)g_source;
	GSList * elem = usb_src->fds;
	GPollFD * g_usb_fd = NULL;

	if(elem){
		do{
			g_usb_fd = elem->data;
			g_source_remove_poll(g_source, g_usb_fd);
			g_slice_free(GPollFD, g_usb_fd);
		}while((elem = g_slist_next(elem)));
	}
	g_slist_free(usb_src->fds);
}

static void usb_fd_added_cb(int fd, short events, void * source){
	libusbSource * usb_src = (libusbSource *)source;
	GPollFD * g_usb_fd = g_slice_new0(GPollFD);

	g_usb_fd->fd = fd;
	if (events & POLLIN)
			g_usb_fd->events |= G_IO_IN;
	if (events & POLLOUT)
			g_usb_fd->events |= G_IO_OUT;

	usb_src->fds = g_slist_prepend(usb_src->fds, g_usb_fd);
	g_source_add_poll((GSource *)usb_src, g_usb_fd);
}

static void usb_fd_removed_cb(int fd, void* source){
	libusbSource * usb_src = (libusbSource *)source;
	GSList * elem = usb_src->fds;
	GPollFD * g_usb_fd = NULL;
	if(g_source_is_destroyed((GSource*)source)){
		return; //finalize has already been run //todo: check if source is destroyed everywhere
	}

	if(!elem)
		return; //error? asked to remove an fd that wasn't being used in a poll

	do{
		g_usb_fd = elem->data;
		if(g_usb_fd->fd == fd){
			g_source_remove_poll((GSource*)source, g_usb_fd);
			g_slice_free(GPollFD, g_usb_fd);
			usb_src->fds = g_slist_delete_link(usb_src->fds, elem);
			break;
		}
	}while((elem = g_slist_next(elem)));
}

static int init_usb_fds(libusbSource * usb_source){
	int numfds = 0;
	const struct libusb_pollfd ** usb_fds = libusb_get_pollfds(usb_source->context);
	if(!usb_fds)
		return -1;

	for(numfds = 0; usb_fds[numfds] != NULL; ++numfds){
		usb_fd_added_cb(usb_fds[numfds]->fd, usb_fds[numfds]->events, usb_source);
	}

	free(usb_fds);
	libusb_set_pollfd_notifiers(usb_source->context, usb_fd_added_cb, usb_fd_removed_cb, usb_source);
	return 0;
}

//todo: does this name confuse with libusb funcs?
libusbSource * libusb_source_new(libusb_context * context){
	static GSourceFuncs usb_funcs = {prepare, check, dispatch, finalize}; //todo is static enough or
	if(!libusb_pollfds_handle_timeouts(context)){                         //or should I malloc
		usb_funcs.prepare = alt_prepare;
	}

	GSource * g_usb_source = g_source_new (&usb_funcs, sizeof(libusbSource));
	libusbSource * usb_source = (libusbSource *) g_usb_source;

	usb_source->fds = NULL; //important to set null because g_source_destroy calls finalize
	usb_source->context = context;
	usb_source->timeout_error = 0;
	usb_source->handle_events_error = 0;

	if(init_usb_fds(usb_source)){
		g_source_destroy((GSource*)usb_source);
		return NULL;
	}

	return usb_source;
}

