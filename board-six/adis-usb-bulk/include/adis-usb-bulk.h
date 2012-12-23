
/*! \file adis-usb-bulk.h
 */

#ifndef _ADIS_DEV_H
#define _ADIS_DEV_H

#include "lpc23xx-usb.h"

#define ADIS_DRDY               (1<<10)

#define ADIS_MAX_PACKET_SIZE    64

#define ADDR_ACC                0x80
#define ADDR_GYR                0x40
#define ADDR_MAG                0x20
#define ADDR_CAC                0x10
#define ADDR_ALL                0xF0

#define IMU_INST(X)             ((X) & 0x0F)
#define IMU_ADDR(X)             ((X) & 0xF0)

#define LE_WORD(x)              ((x)&0xFF),((x)>>8)

typedef enum {
	INST_RESET       = 0x01,
	INST_GO          = 0x02,
	INST_STOP        = 0x03,
	INST_SET_SPEED   = 0x04
} adis_usb_bulk_instruction;

typedef enum {
	CTRL_OUT_EP      = 0x00,
	BULK0_IN_EP      = 0x82
} adis_usb_endpoint;

void adis_dev_isr(void) __attribute__ ((interrupt("IRQ")));

static const uint8_t adis_usb_bulk_descriptor[] = {

    // device descriptor
    0x12,
    DESC_DEVICE,
    LE_WORD(0x0200),                    // bcdUSB
    0xFF,                               // bDeviceClass
    0xFF,                               // bDeviceSubClass
    0xFF,                               // bDeviceProtocol
    MAX_PACKET_SIZE0,                   // bMaxPacketSize
    LE_WORD(0xFFFF),                    // idVendor
    LE_WORD(0x0009),                    // idProduct
    LE_WORD(0x0100),                    // bcdDevice
    0x01,                               // iManufacturer
    0x02,                               // iProduct
    0x03,                               // iSerialNumber
    0x01,                               // bNumConfigurations

    // configuration descriptor
    0x09,
    DESC_CONFIGURATION,
    LE_WORD(25),                        // wTotalLength
    0x01,                               // bNumInterfaces
    0x01,                               // bConfigurationValue
    0x00,                               // iConfiguration
    0x80,                               // bmAttributes
    0xF0,                               // bMaxPower

    // data class interface descriptor
    0x09,
    DESC_INTERFACE,
    0x00,                                // bInterfaceNumber
    0x00,                                // bAlternateSetting
    0x01,                                // bNumEndPoints
    0xFF,                                // bInterfaceClass = vendor
    0xFF,                                // bInterfaceSubClass
    0xFF,                                // bInterfaceProtocol
    0x00,                                // iInterface
    // Accelerometer isoc in ep
    0x07,
    DESC_ENDPOINT,
    BULK0_IN_EP,                           // bEndpointAddress
    0x02,                                  // bmAttributes = bulk
    LE_WORD(2* ADIS_NUM_BURSTREAD_REGS),   // wMaxPacketSize
    0x01,                                  // bInterval

    // string descriptors
    0x04,                                //bLength
    DESC_STRING,                         //bDescriptorType
    LE_WORD(0x0409),                     //wLANGID[0]

    0x0A,
    DESC_STRING,
    'P', 0, 'S', 0, 'A', 0, 'S', 0,

    0x12,
    DESC_STRING,
    'A', 0, 'D', 0, 'I', 0, 'S', 0, ' ', 0, 'I', 0, 'M', 0, 'U', 0,

    0x1A,
    DESC_STRING,
    'G', 0, 'F', 0, 'E', 0, ' ', 0, 'D', 0, 'e', 0, 'c', 0, ' ', 0, '2', 0,
    '0', 0, '1', 0, '2', 0,

    // terminating zero
    0
};


#endif

