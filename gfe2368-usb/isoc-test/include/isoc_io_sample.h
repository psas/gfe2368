/*! \file isoc_io_sample.h
*/

#ifndef _ISOC_IO_SAMPLE_H
#define _ISOC_IO_SAMPLE_H

#include <stdint.h>

#define INT_IN_EP                       0x81

#define ISOC_OUT_EP                     0x06
#define ISOC_IN_EP                      0x83

#define MAX_PACKET_SIZE                 128

#define LE_WORD(x)                      ((x)&0xFF),((x)>>8)

#define BYTES_PER_ISOC_FRAME            4

#define INT_VECT_NUM                    0

#define ISOC_OUTPUT_DATA_BUFFER_SIZE    1024

__attribute__ ((aligned(4))) uint32_t inputIsocDataBuffer[(BYTES_PER_ISOC_FRAME/4)];

volatile                     uint8_t  outputIsocDataBuffer[ISOC_OUTPUT_DATA_BUFFER_SIZE];

//int isConnectedFlag = 0;
//
//uint8_t bDevStat         = 0;

// static uint8_t abClassReqData[8];

// static void USBIntHandler(void) __attribute__ ((interrupt(IRQ), naked));

static const uint8_t abDescriptors[] = {

    // device descriptor
    0x12,
    DESC_DEVICE,
    LE_WORD(0x0101),                        // bcdUSB
    0x02,                                   // bDeviceClass
    0x00,                                   // bDeviceSubClass
    0x00,                                   // bDeviceProtocol
    MAX_PACKET_SIZE0,                       // bMaxPacketSize
    LE_WORD(0xFFFF),                        // idVendor
    LE_WORD(0x0005),                        // idProduct
    LE_WORD(0x0100),                        // bcdDevice
    0x01,                                   // iManufacturer
    0x02,                                   // iProduct
    0x03,                                   // iSerialNumber
    0x01,                                   // bNumConfigurations

    // configuration descriptor
    0x09,
    DESC_CONFIGURATION,
    LE_WORD(32), //sizeof(this configuration descriptor) + sizeof(all interfaces defined)   //LE_WORD(67),                          // wTotalLength
    0x01, //0x02,                           // bNumInterfaces
    0x01,                                   // bConfigurationValue
    0x00,                                   // iConfiguration
    0xC0,                                   // bmAttributes
    0x32,                                   // bMaxPower

    // data class interface descriptor   9+7+7=23
    0x09,
    DESC_INTERFACE,
    0x00,                                   // bInterfaceNumber
    0x00,                                   // bAlternateSetting
    0x02,//DC                               // bNumEndPoints
    0xFF,// 0x0A,                           // bInterfaceClass = data
    0x00,                                   // bInterfaceSubClass
    0x00,                                   // bInterfaceProtocol
    0x00,                                   // iInterface

    // data EP OUT
    0x07,
    DESC_ENDPOINT,
    ISOC_OUT_EP,                            // bEndpointAddress
    0x0D,                                   // bmAttributes = isoc, syncronous, data endpoint
    LE_WORD(MAX_PACKET_SIZE),               // wMaxPacketSize
    0x01,                                   // bInterval    

    // data EP OUT
    0x07,
    DESC_ENDPOINT,
    ISOC_IN_EP,                             // bEndpointAddress
    0x0D,                                   // bmAttributes = isoc, syncronous, data endpoint
    LE_WORD(MAX_PACKET_SIZE),               // wMaxPacketSize
    0x01,                                   // bInterval    

    // string descriptors
    0x04,
    DESC_STRING,
    LE_WORD(0x0409),

    0x0E,
    DESC_STRING,
    'L', 0, 'P', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,

    0x14,
    DESC_STRING,
    'U', 0, 'S', 0, 'B', 0, 'S', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0,

    0x12,
    DESC_STRING,
    'D', 0, 'E', 0, 'A', 0, 'D', 0, 'C', 0, '0', 0, 'D', 0, 'E', 0,

    // terminating zero
    0
};



#endif

