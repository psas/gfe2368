

#ifndef USB_SPEED_TEST_H_
#define USB_SPEED_TEST_H_

#include "lpc23xx-types.h"

static void GPIO_isr(void)      __attribute__ ((interrupt("IRQ")));
static void USBIntHandler(void) __attribute__ ((interrupt("IRQ")));

#define VIC_GPIO_BIT        (17)
#define ENABLE_GPIO_INT     (VICIntEnable |= (1<<VIC_GPIO_BIT))
#define DISABLE_GPIO_INT    (VICIntEnClr   = (1<<VIC_GPIO_BIT))
#define RAISE_GPIO_INT      (VICSoftInt   |= (1<<VIC_GPIO_BIT))
#define CLR_SW_GPIO_INT     (VICSoftIntClr = (1<<VIC_GPIO_BIT))


#define CTRL_IN_EP				0x80
#define CTRL_OUT_EP				0x00
#define INTR_IN_EP              0x81
#define INTR_OUT_EP				0x01
#define BULK_IN_EP              0x82
#define BULK_OUT_EP             0x02
#define ISOC_IN_EP				0x83
#define ISOC_OUT_EP				0x03

#define MAX_PACKET_SIZE 64

#define LE_WORD(x)              ((x)&0xFF),((x)>>8)

#define USB_TEST_IN 16
#define USB_TEST_OUT 17


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
	LE_WORD(0x0005),                    // idProduct
	LE_WORD(0x0050),                    // bcdDevice
	0x01,                               // iManufacturer
	0x02,                               // iProduct
	0x03,                               // iSerialNumber
	0x01,                               // bNumConfigurations

    // configuration descriptor
    0x09,
    DESC_CONFIGURATION,
    LE_WORD(67),                        // wTotalLength       todo: set correctly
    0x04,                               // bNumInterfaces
    0x01,                               // bConfigurationValue
    0x00,                               // iConfiguration
    0x80,                               // bmAttributes
    0xFF,                               // bMaxPower          todo: how much do I actually use?

    // test interface
	0x09,
	DESC_INTERFACE,
	0x01,                               // bInterfaceNumber
	0x00,                               // bAlternateSetting
	0x06,                               // bNumEndPoints
	0xff,                               // bInterfaceClass
	0x00,                               // bInterfaceSubClass
	0x00,                               // bInterfaceProtocol
	0x00,                               // iInterface

	// interrupt EP in
    0x07,
    DESC_ENDPOINT,
    INTR_IN_EP,                          // bEndpointAddress
    0x03,                                // bmAttributes = interrupt
    LE_WORD(1),           				 // wMaxPacketSize
    0x01,                                // bInterval

    // interrupt EP out
    0x07,
    DESC_ENDPOINT,
    INTR_OUT_EP,                         // bEndpointAddress
    0x03,                                // bmAttributes = interrupt
    LE_WORD(1),            				 // wMaxPacketSize
    0x01,                                // bInterval

	// bulk EP in
    0x07,
    DESC_ENDPOINT,
    BULK_IN_EP,                          // bEndpointAddress
    0x02,                                // bmAttributes = bulk
    LE_WORD(1),            				 // wMaxPacketSize
    0x00,                                // bInterval

    // bulk EP out
    0x07,
    DESC_ENDPOINT,
    BULK_OUT_EP,                         // bEndpointAddress
    0x02,                                // bmAttributes = bulk
    LE_WORD(1),          			     // wMaxPacketSize
    0x00,                                // bInterval

	// isochronous EP in
    0x07,
    DESC_ENDPOINT,
    ISOC_IN_EP,                          // bEndpointAddress
    0x01,                                // bmAttributes = isochronous
    LE_WORD(1),           				 // wMaxPacketSize
    0x01,                                // bInterval

    // isochronus EP out
    0x07,
    DESC_ENDPOINT,
    ISOC_OUT_EP,                         // bEndpointAddress
    0x01,                                // bmAttributes = isochronous
    LE_WORD(1),           				 // wMaxPacketSize
    0x01,                                // bInterval

    // string descriptors
    0x04,
    DESC_STRING,
    LE_WORD(0x0409),

    0x0A,
    DESC_STRING,
    'P', 0, 'S', 0, 'A', 0, 'S', 0,

    0x14,
    DESC_STRING,
    'U', 0, 'S', 0, 'B', 0, 'S', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0,

    0x16,
    DESC_STRING,
    'l', 0, 'i', 0, 'b', 0, 'l', 0, 'p', 0, 'c', 0, '2', 0, '3', 0, 'x', 0, 'x', 0,

    // terminating zero
    0
};

#endif /* USB_SPEED_TEST_H_ */
