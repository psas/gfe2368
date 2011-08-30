
/*
 * dp-bulkusb.c
 */

/**
 * \mainpage dp-bulkusb.c
 *
 * \section intro Introduction
 *
 * dp-bulkusb is an application to test the datapath to a PSAS
 * GFE node performing as a LPC2368 based USB device.
 *
 * It is also an investigation of the libusb1.0 library. See
 *<a href="http://libusb.sourceforge.net">libusb homepage</a>.
 *
 * This program will communicate with the device in bulk USB mode.
 *
 * To complile the code type:

 \code
 make
 \endcode

 * It is also an experiment in using the 'doxygen' tools to document
 * the code.
 */

#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <syslog.h>

#include <unistd.h>  // getpid()
#include <errno.h>
#include <time.h>
#include <termios.h>    // terminal

#include "libusb-1.0/libusb.h"

#include "parse-args.h"
#include "dp-bulkusb.h"


/**
 * Do something with data read in from bulk
 */
static int save_bulk_in(uint8_t *data, int32_t length) {
    FILE *fd;
    char filename[64];
    size_t ignore;
    int  i = 0;

    sprintf(filename, "dp_bulk_in.data");
    fd = fopen(filename, "a");
    if (!fd) {
        fprintf(stderr, "save_bulk_in: Open file %s fail.", filename);
        return(-1);
    }
    ignore = fwrite(data, 1, length, fd);
    fclose(fd);

    printf("*** bulk in xfer ***\n");
    for(i=0; i<length; ++i) {
        printf("%x", *data);
        if(i%4 == 0){
            printf("\n");
        }
    }
    return 0;
}

/**
 * callback for asynch. bulk_in xfer
 */
static void cbk_bulk_in(struct libusb_transfer* xfer) {

    if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
        fprintf(stderr, "cbk_bulk_in: transfer status: %d\n", xfer->status);
        libusb_free_transfer(xfer);
        bulk_xfer_in = NULL;
        exit_test = 1;
        return;
    }

    DBG("cbk_bulk_in: In callback...\n");

    save_bulk_in(bulk_buf_in, xfer->actual_length);

    if (libusb_submit_transfer(xfer) < 0) {
        fprintf(stderr, "cbk_bulk_in: submit next transfer error\n");
        libusb_free_transfer(xfer);
        bulk_xfer_in = NULL;
        exit_test = 1;
        return;
    }
    return;
}

/**
 * Finds the handle of a device with idvendor, idproduct.
 *
 * \param idvendor Vendor Id
 * \param idproduct Product Id
 *
 * \returns
 *     - 0 on success
 *     - -EIO on failure
 */
static int find_gfe_device(uint16_t idvendor, uint16_t idproduct) {

    devh = libusb_open_device_with_vid_pid(NULL, idvendor, idproduct);

    return devh ? 0 : -EIO;
}

/**
 * initialize data structures bulk_xfer_in
 */
static int init_bulk_xfer_in(struct libusb_transfer* bulk_xfer_in) {

    bulk_xfer_in = libusb_alloc_transfer(0);
    if (!bulk_xfer_in) {
        fprintf(stderr,"init_bulk_xfer: Failed to allocate structure.\n");
        return -ENOMEM;
    }

    libusb_fill_bulk_transfer(bulk_xfer_in, devh, EP_BULK_IN, bulk_buf_in,
            sizeof(bulk_buf_in), cbk_bulk_in, NULL, 0);

    return 0;
}

/**
 * clean up the interface before exit
 */
void clean_interface() {
    libusb_release_interface(devh, 0);
    libusb_close(devh);
    libusb_exit(NULL);
}

/**
 * initialize the standard input
 *
 * @param orig_stdin_tios Original termios structure
 *
 * @return
 *  - 0 success
 *  - -1 Fail
 */
int init_stdin(struct termios* orig_stdin_tios) {

    struct termios      tios;

    // Get tios
    if (tcgetattr(0, &tios)){
        fprintf(stderr,"init_stdin: Error getting current terminal settings\n");
        return (-1);
    } else {
        tcgetattr(0, orig_stdin_tios);
    }

    tios.c_lflag        &= ~ICANON;
    tios.c_lflag        |= ECHO;

    tios.c_cc[VMIN]     = VMIN_VALUE;
    tios.c_cc[VTIME]    = VTIME_VALUE;

    if (tcsetattr(0, TCSAFLUSH, &tios)){
        fprintf(stderr,"init_stdin: Error applying terminal settings\n");
        return (-1);
    }
    return(0);
}

/**
 * return the standard input to original status
 *
 * @param orig_stdin_tios Original termios structure
 *
 * @return
 *  - 0 success
 *  - -1 failure
 */
int reset_stdin(struct termios* orig_stdin_tios) {

    if(tcsetattr(0, TCSAFLUSH, orig_stdin_tios)) {
        syslog(LOG_CRIT, "reset_stdin: failed to reset attributes on stdin.\n");
        return(-1);
    };
    return(0);
}

/**
 *  task to read and write bulk usb data while
 *  reading commands from stdin
 */
void dp_task() {
    time_t                begin, end;
    double                totalsecs;
    double                avgrate;

    int32_t                     ret          = 1;
    int32_t             bytecount    = 0;

    int32_t             bytes_stdin  = 0;
    int32_t                             bytes_out    = 0;

    uint8_t             value_stdin  = 0;
    struct termios      orig_stdin_tios;

    if(init_stdin(&orig_stdin_tios) != 0) {
        fprintf(stderr, "datapath_task: Failed to init stdin.\n");
        exit(EXIT_FAILURE);
    };

    ret = libusb_init(NULL);
    if (ret < 0) {
        fprintf(stderr, "dp_task: failed to init libusb..exiting\n");
        exit(EXIT_FAILURE);
    }

    ret = find_gfe_device(IDVENDOR, IDPRODUCT);
    if(ret < 0) {
        fprintf(stderr, "dp_task: Failed to find gfe device.");
    }

    // check if kernel already has claimed device:
    ret = libusb_kernel_driver_active(devh,0);
    if(ret == 1) {
        fprintf(stderr, "Kernel has device. Forcing kernel to release device.\n");
        ret = libusb_detach_kernel_driver(devh, 0);
        if(ret != 0) {
            fprintf(stderr, "dp_task: detach_kernel_driver failure, return: %i", ret);
            clean_interface();
            exit(EXIT_FAILURE);
        }
    } else if (ret == LIBUSB_ERROR_NO_DEVICE) {
        fprintf(stderr, "No device found\n");
        clean_interface();
        exit(EXIT_FAILURE);
    } else if (ret != 0) {
        fprintf(stderr, "dp_task: kernel_driver active returned: %i", ret);
        clean_interface();
        exit(EXIT_FAILURE);
    } else {}


    ret = libusb_claim_interface(devh, 0);
    if (ret < 0) {
        fprintf(stderr, "dp_task: claim_interface error: %d\n", ret);
        clean_interface();
        exit(EXIT_FAILURE);
    }
    DBG("dp_task: claimed interface\n");

    ret = init_bulk_xfer_in(bulk_xfer_in);
    if (ret < 0) {
        fprintf(stderr, "dp_task: init_bulk_xfer error: %d\n", ret);
        clean_interface();
        exit(EXIT_FAILURE);
    }

    printf("\nOptions: (s)-stop, (r)-reset, (g)-go, (q)-quit\n");
    ret = reset_stdin(&orig_stdin_tios);
    if(ret < 0) {
        fprintf(stderr, "dp_task: failed to reset stdin\n");
    }
    while (!exit_test) {
        // get info from console (user)
        bytes_stdin = read(0, &value_stdin, 1);
        if (bytes_stdin > 0) {
            printf("\nYou typed: %c", (char) value_stdin);
            printf("\nOptions: (s)-stop, (r)-reset, (g)-go, (q)-quit\n");
            switch(value_stdin) {
                case 'r':
                    printf("\tRESET\n");
                    break;
                case 'g':
                    printf("\tGO\n");
                    break;
                case 's':
                    printf("\tSTOP\n");
                    break;
                default:
                    break;
            }
            printf("\n");
            if(value_stdin == 'g') {
                time(&begin);
                bytecount = 0;
            } else if(value_stdin == 's') {
                time(&end);
                totalsecs = difftime (end, begin);
                avgrate   = bytecount/totalsecs;
                printf("Average rate:\t%5.2f bytes/sec.\t%5.2f bits/sec.\n", avgrate, avgrate*8);
            }

            DBG("Sending data: 0x%x\t%c\n", value_stdin, (char) value_stdin);
            ret = libusb_bulk_transfer(devh, EP_BULK_OUT, &value_stdin, 1, &bytes_out, 0);

            if(ret != 0) {
                fprintf(stderr, "\n*** Write bulk failed with return: %i, bytes_out are %d\n", ret, bytes_out);
                switch(ret) {
                    case LIBUSB_ERROR_TIMEOUT: 
                        fprintf(stderr, "\n*** ERROR_TIMEOUT ***\n");
                        break;
                    case LIBUSB_ERROR_PIPE: 
                        fprintf(stderr, "\n*** ERROR_PIPE ***\n");
                        break;
                    case LIBUSB_ERROR_OVERFLOW: 
                        fprintf(stderr, "\n*** ERROR_OVERFLOW ***\n");
                        break;
                    case LIBUSB_ERROR_NO_DEVICE: 
                        fprintf(stderr, "\n*** ERROR_NO_DEVICE ***\n");
                        break;
                    default:
                        fprintf(stderr, "\n*** other error %i ***\n", ret);
                        break;
                }
            }

            if(value_stdin == 'q') {
                printf("Quitting.\n");
                exit_test=1;
                DBG("Reset stdin.\n");
                ret = reset_stdin(&orig_stdin_tios);
                if(ret < 0) {
                    fprintf(stderr, "dp_task: failed to reset stdin\n");
                }
                break;
            }

        }
        DBG("handle events\n");
        ret = libusb_handle_events(NULL);
        DBG("handle events out\n");
        if (ret < 0) {
            fprintf(stderr, "dp_task: handle_events error: %d\n", ret);
            clean_interface();
            exit(EXIT_FAILURE);
        }
    }
}


int main(int argc, char* argv[]) {
    struct stat st;
    char f[100];
    int x;
    printf("**Starting %s\n", argv[0]);

    if(stat("dp_bulk_in.data",&st) == 0) {
        printf("moving dp_bulk_in.data to backup.\n");
        sprintf(f, "mv dp_bulk_in.data dp_bulk_in.data%d", getpid());
        DBG("calling system(%s)\n", f);
        x = system(f);
        DBG("system() returned %d\n", x);
        if (x != EXIT_SUCCESS) {
            fprintf(stderr,"main: Backing up old data file failed.\n");
            exit(EXIT_FAILURE);
        }
    }

    dp_task();

    printf("\n** %s Done. **\n\n", argv[0]);

    clean_interface();
    return(0);

}


