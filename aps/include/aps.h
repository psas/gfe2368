
#include "lpc23xx-usb.h"
#define CTRL_IN_EP              0x80
#define CTRL_OUT_EP             0x00

#define IOUT_PIN 23
#define ACOK_PIN 11
#define FC_SPS_PIN 5
#define ATV_SPS_PIN 6
#define RC_POWER_PIN 7
#define ROCKET_READY_PIN 8
#define WIFI_POWER_PIN 9
#define RC_TETHER 15

#define MAX_PACKET_SIZE 64

#define LE_WORD(x)              ((x)&0xFF),((x)>>8)


#define READ_REGISTER 0
#define WRITE_REGISTER 1

void GPIO_isr(void)__attribute__ ((interrupt("IRQ")));

//todo: functions/macros to create/spell check descriptors
static const uint8_t abDescriptors[] = {
    // device descriptor
    0x12,
    DESC_DEVICE,
    LE_WORD(0x0200),                    // bcdUSB
    0xff,                               // bDeviceClass
    0x00,                               // bDeviceSubClass
    0x00,                               // bDeviceProtocol
    MAX_PACKET_SIZE0,                   // bMaxPacketSize
    LE_WORD(0xFFFF),                    // idVendor
    LE_WORD(0x0006),                    // idProduct
    LE_WORD(0x0050),                    // bcdDevice
    0x01,                               // iManufacturer
    0x02,                               // iProduct
    0x03,                               // iSerialNumber
    0x01,                               // bNumConfigurations

    // configuration descriptor
    0x09,
    DESC_CONFIGURATION,
    LE_WORD(18),                        // wTotalLength
    0x01,                               // bNumInterfaces
    0x01,                               // bConfigurationValue
    0x00,                               // iConfiguration
    0x80,                               // bmAttributes
    0xF0,                               // bMaxPower          todo: how much do I actually use?

    // test interface
    0x09,
    DESC_INTERFACE,
    0x00,                               // bInterfaceNumber
    0x00,                               // bAlternateSetting
    0x00,                               // bNumEndPoints
    0xff,                               // bInterfaceClass
    0x00,                               // bInterfaceSubClass
    0x00,                               // bInterfaceProtocol
    0x00,                               // iInterface

    // string descriptors
    0x04,
    DESC_STRING,
    LE_WORD(0x0409),

    0x0A,
    DESC_STRING,
    'P', 0, 'S', 0, 'A', 0, 'S', 0,

    0x14,
    DESC_STRING,
    'U', 0, 'S', 0, 'B', 0, 'A', 0, 'P', 0, 'S', 0, 't', 0, 's', 0, 't', 0,

    0x16,
    DESC_STRING,
    'l', 0, 'i', 0, 'b', 0, 'l', 0, 'p', 0, 'c', 0, '2', 0, '3', 0, 'x', 0, 'x', 0,

    // terminating zero
    0
};
