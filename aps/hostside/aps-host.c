/*
 * apu-host.c
 *
 */


//todo: amazeballz mode
#include <stdlib.h>
#include <stdio.h>
#include <libusb.h>
#include <time.h>
#include <poll.h>
#include <stdbool.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <signal.h>
#include <argp.h>

#include "libusb-util.h"
#include "aps-host.h"

#include "lpc23xx.h"

#define CTRL_IN_EP              0x80
#define CTRL_OUT_EP             0x00
#define INTR_IN_EP              0x81
#define INTR_OUT_EP             0x01
#define BULK_IN_EP              0x82
#define BULK_OUT_EP             0x02
#define ISOC_IN_EP              0x83
#define ISOC_OUT_EP             0x03
#define READ_REGISTER 0
#define WRITE_REGISTER 1

#define RED_LED_PIN          19
#define BLUE_LED_PIN         22
#define GREEN_LED_PIN        25


#define IOUT_PIN 23
#define ACOK_PIN 11
#define FC_SPS_PIN 5
#define ATV_SPS_PIN 6
#define RC_POWER_PIN 7
#define ROCKET_READY_PIN 8
#define WIFI_POWER_PIN 9
#define RC_TETHER 15

libusb_context * apu_host = NULL;
libusb_device_handle * apu_handle = NULL;
uint32_t gpio_set;
uint32_t gpio_clear;

bool is_apu_device(libusb_device * device){
    struct libusb_device_descriptor descr;
    int retErr = libusb_get_device_descriptor(device, &descr);
    if(retErr){
        print_libusb_error(retErr,"is_imu_device libusb_get_device_descriptor");
        return false;
    }

    if(descr.idVendor == 0xFFFF && descr.idProduct == 0x0006){
        //todo: more ID methods
        return true;
    }

    return false;
}

void set_port(int port, uint32_t val){
    int setport = 0x80;
    unsigned char data[64];
    int usb_err;
    data[0] = (val & 0xFF<<0)>>0;
    data[1] = (val & 0xFF<<8)>>8;
    data[2] = (val & 0xFF<<16)>>16;
    data[3] = (val & 0xFF<<24)>>24;
    usb_err = libusb_control_transfer(apu_handle,
            LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
            setport | port, 0, 0, data, 4, 2000);
    if(usb_err < 0){
        print_libusb_error(usb_err, "set_port");
    }
    if(usb_err != 4){
        printf("set_port: Didn't send correct number of bytes");
    }
}

void clear_port(int port, uint32_t val){
    int clearport = 0x40;
    unsigned char data[64];
    int usb_err;
    data[0] = (val & 0xFF<<0)>>0;
    data[1] = (val & 0xFF<<8)>>8;
    data[2] = (val & 0xFF<<16)>>16;
    data[3] = (val & 0xFF<<24)>>24;
    usb_err = libusb_control_transfer(apu_handle,
            LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
            clearport | port, 0, 0, data, 4, 2000);
    if(usb_err < 0){
        print_libusb_error(usb_err, "set_port");
    }
    if(usb_err != 4){
        printf("set_port: Didn't send correct number of bytes");
    }
}

void read_gpio(uint32_t * ports){
    int readport = 0x00, i;
    unsigned char data[64];
    int usb_err;
    usb_err = libusb_control_transfer(apu_handle,
            LIBUSB_RECIPIENT_OTHER | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
            readport, 0, 0, data, 4*5, 2000);
    if(usb_err < 0){
        print_libusb_error(usb_err, "read_gpio");
    }
    if(usb_err != 4*5){
        printf("read_gpio: Didn't read correct number of bytes");
    }
    for(i = 0; i < 5; ++i){
        ports[i] = (data[4*i]) | (data[4*i+1]<<8) | (data[4*i+2]<<16) | (data[4*i+3]<<24);
    }
}

error_t parse_opt (int key, char *arg, struct argp_state *state)
{
  switch (key)
    {
    case 'a':
        if(arg == 0){
            gpio_set = ((1<<ATV_SPS_PIN) | (1<<RC_POWER_PIN) |
                        (1<<WIFI_POWER_PIN));
            gpio_clear = 0;
        }else if (!strcmp(arg, "off")){
            gpio_clear = ((1<<ATV_SPS_PIN) | (1<<RC_POWER_PIN) |
                          (1<<WIFI_POWER_PIN));
            gpio_set = 0;
        }
        break;
    case 'w':
        if(arg == 0){
            gpio_set |= (1<<WIFI_POWER_PIN);
            gpio_clear &= ~(1<<WIFI_POWER_PIN);
        }else if (!strcmp(arg, "off")){
            gpio_clear |= (1<<WIFI_POWER_PIN);
            gpio_set &= ~(1<<WIFI_POWER_PIN);
        }
        break;
    case 't':
        if(arg == 0){
            gpio_set |= (1<<ATV_SPS_PIN);
            gpio_clear &= ~(1<<ATV_SPS_PIN);
        }else if (!strcmp(arg, "off")){
            gpio_clear |= (1<<ATV_SPS_PIN);
            gpio_set &= ~(1<<ATV_SPS_PIN);
        }
        break;
    case 'r':
        if(arg == 0){
            gpio_set |= (1<<RC_POWER_PIN);
            gpio_clear &= ~(1<<RC_POWER_PIN);
        }else if (!strcmp(arg, "off")){
            gpio_clear |= (1<<RC_POWER_PIN);
            gpio_set &= ~(1<<RC_POWER_PIN);
        }
        break;
    case 'c':
        if(arg == 0){
            gpio_set |= (1<<RC_TETHER);
            gpio_clear &= ~(1<<RC_TETHER);
        }else if (!strcmp(arg, "off")){
            gpio_clear |= (1<<RC_TETHER);
            gpio_set &= ~(1<<RC_TETHER);
        }
        break;
    case 'f':
        if(arg == 0){
            gpio_set |= (1<<FC_SPS_PIN);
            gpio_clear &= ~(1<<FC_SPS_PIN);
        }else if (!strcmp(arg, "off")){
            gpio_clear |= (1<<FC_SPS_PIN);
            gpio_set &= ~(1<<FC_SPS_PIN);
        }
        break;
    case 'k':
        if(arg == 0){
            gpio_set |= (1<<ROCKET_READY_PIN);
            gpio_clear &= ~(1<<ROCKET_READY_PIN);
        }else if (!strcmp(arg, "off")){
            gpio_clear |= (1<<ROCKET_READY_PIN);
            gpio_set &= ~(1<<ROCKET_READY_PIN);
        }
        break;
    default:
      return ARGP_ERR_UNKNOWN;
    }
  return 0;
}


int main(int argc, char*argv[]){
    struct argp_option all = {
            .name = "all",
            .key = 'a',
            .arg = "off",
            .flags = OPTION_ARG_OPTIONAL,
            .doc = "Turns on/off ALL the things",
    };
    struct argp_option wifi = {
            .name = "wifi",
            .key = 'w',
            .arg = "off",
            .flags = OPTION_ARG_OPTIONAL,
            .doc = "Turns on/off the Wifi amp",
    };
    struct argp_option atv = {
            .name = "atv",
            .key = 't',
            .arg = "off",
            .flags = OPTION_ARG_OPTIONAL,
            .doc = "Turns on/off the ATV amp",
    };
    struct argp_option rc = {
            .name = "rc",
            .key = 'r',
            .arg = "off",
            .flags = OPTION_ARG_OPTIONAL,
            .doc = "Turns on/off the roll control",
    };
    struct argp_option rct = {
            .name = "rct",
            .key = 'c',
            .arg = "off",
            .flags = OPTION_ARG_OPTIONAL,
            .doc = "Turns on/off the roll control tether",
    };
    struct argp_option fc = {
            .name = "fc",
            .key = 'f',
            .arg = "off",
            .flags = OPTION_ARG_OPTIONAL,
            .doc = "Turns off the flight computer",
    };
    struct argp_option rocket_ready = {
            .name = "rocket-ready",
            .key = 'k',
            .arg = "off",
            .flags = OPTION_ARG_OPTIONAL,
            .doc = "Turns on/off rocket ready",
    };

    struct argp_option opts[] = {all, wifi, atv, rc, rct, fc, rocket_ready, {0}};

    struct argp parse_strct = {
        .options = opts,
        .parser = parse_opt,
        .doc = "I LIEK TURTLES"
    };

    argp_parse (&parse_strct, argc, argv, 0, 0, 0);

    int usbErr = 0, i;
    char in;
    int iface_nums[1]= {0};
    uint32_t ports[5] = {0};

    usbErr = libusb_init(&apu_host);
    if(usbErr){
        print_libusb_error(usbErr, "libusb_init");
        exit(EXIT_FAILURE);
    }
    libusb_set_debug(apu_host, 3);
    //get the usb device
    printf("GPIO-SET: %x\n", gpio_set);
    printf("GPIO-CLR: %x\n", gpio_clear);
    do{
        apu_handle = open_usb_device_handle(apu_host, is_apu_device, iface_nums, 1);
        if(!apu_handle){
            printf("**device handle acquisition error, retry (y/n)?\n");
            in = getchar();
            while(getchar() != '\n');
            if(in != 'y'){
                printf("quitting\n");
                //libusbSource_free(usb_source);
                libusb_exit(apu_host);
                exit(EXIT_FAILURE);
            }
        }
    }while(!apu_handle);



    set_port(0, gpio_set);
    clear_port(0, gpio_clear);
    read_gpio(ports);
    for(i = 0; i < 5; ++i)
        printf("port %d: %x\n",i, ports[i]);

    libusb_close(apu_handle);
    libusb_exit(apu_host);
    return 0;
}
