
/*! \file adis-dev.h
 */



#ifndef _ADIS_DEV_H
#define _ADIS_DEV_H

#include "lpc23xx-usb.h"

#define BULK0_IN_EP 0x82

#define ADIS_DRDY (1<<10)

#define CTRL_OUT_EP             0x00
#define ADDR_ACC                0x80
#define ADDR_GYR                0x40
#define ADDR_MAG                0x20
#define ADDR_CAC                0x10
#define ADDR_ALL                0xF0
#define INST_RESET              0x01
#define INST_GO                 0x02
#define INST_STOP               0x03
#define INST_SET_SPEED          0x04

#define IMU_INST(X)             ((X) & 0x0F)
#define IMU_ADDR(X)             ((X) & 0xF0)

#define ADIS_PACKET_LENGTH 24
void adis_isr(void) __attribute__ ((interrupt("IRQ")));

#define MAX_PACKET_SIZE 64

#define LE_WORD(x)              ((x)&0xFF),((x)>>8)


static const uint8_t imu_descriptor[] = {

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
    BULK0_IN_EP,                         // bEndpointAddress
    0x02,                                // bmAttributes = bulk
    LE_WORD(ADIS_PACKET_LENGTH),         // wMaxPacketSize
    0x01,                                // bInterval

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
    'A', 0, 'V', 0, '3', 0, ' ', 0, 'O', 0, 'c', 0, 't', 0, ' ', 0, '2', 0,
    '0', 0, '1', 0, '2', 0,

    // terminating zero
    0
};

#endif

