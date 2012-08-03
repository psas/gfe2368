/*
 * usb-speed-test.c
 *
 *  Created on: Jul 24, 2012
 *      Author: theo
 */



/*
 * datapath-test.c
 *
 * Test the datapath from a sensor to
 * the LPC to USB to the FC and from the FC
 * to the LPC.
 *
 */

//#define DEBUG_USB

#include <stdio.h>                      // EOF
#include <string.h>                     // memcpy
#include <limits.h>
#include <stdint.h>

#include "lpc23xx.h"
#include "lpc23xx-debug.h"
#include "printf-lpc.h"
#include "lpc23xx-mam.h"
#include "lpc23xx-pll.h"
#include "lpc23xx-uart.h"
#include "lpc23xx-util.h"
#include "lpc23xx-vic.h"
#include "lpc23xx-i2c.h"
#include "lpc23xx-timer.h"
#include "lpc23xx-binsem.h"
#include "usbapi.h"
#include "serial-fifo.h"
#include "gfe2368-util.h"
#include "usb-speed-test.h"
#include "imu-device-host-interface.h"

static uint8_t          abBulkBuf[64];
static uint8_t          abClassReqData[8];

#define CTRL_REQ 1
#define INTR_REQ 2
#define BULK_REQ 3
#define ISOC_REQ 4
#define STOPPED  0
static int mode = STOPPED;


static void ToggleUSBTestOut(){
	if(FIO0SET & 1<<USB_TEST_OUT)
		FIO0CLR = 1<<USB_TEST_OUT;
	else
		FIO0SET = 1<<USB_TEST_OUT;
};
//static void PulseUSBTestOut(){
//	int i;
//	ToggleUSBTestOut();
//	for(i = 0; i < 1000; ++i); //todo: nonblocking, specify length of pulse in time instead of cycles.
//	ToggleUSBTestOut();
//}


static void BulkIn(uint8_t bEP, uint8_t bEPStatus){
	if(bEPStatus & EP_STATUS_NACKED){
		ToggleUSBTestOut();
    }
}
static void BulkOut(uint8_t bEP, uint8_t bEPStatus){
    int iLen;

    // get data from USB into intermediate buffer
    iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));

}
static void InterruptIn(uint8_t bEP, uint8_t bEPStatus){

}
static void InterruptOut(uint8_t bEP, uint8_t bEPStatus){

}
static void IsochronousIn(uint8_t bEP, uint8_t bEPStatus){

}
static void IsochronousOut(uint8_t bEP, uint8_t bEPStatus){

}

static BOOL HandleVendorRequest(TSetupPacket *pSetup, int *piLen, uint8_t **ppbData){
	if(REQTYPE_GET_DIR(pSetup->bmRequestType)){
		//device-to-host

	}else{
		//host-to-device
		mode = pSetup->bRequest;
	}

	return TRUE;
}

/**
  Interrupt handler

  Simply calls the USB ISR, then signals end of interrupt to VIC
  */
static void USBIntHandler(void)
{
    USBHwISR();
    VICAddress = 0x00;    // dummy write to VIC to signal end of ISR
}

/**
  USB frame interrupt handler

  Called every milisecond by the hardware driver.

  This function is responsible for sending the first of a chain of packets
  to the host. A chain is always terminated by a short packet, either a
  packet shorter than the maximum packet size or a zero-length packet
  (as required by the windows usbser.sys driver).

*/
static void USBFrameHandler(uint16_t wFrame){

}

/**
  USB device status handler

  Resets state machine when a USB reset is received.
  */
static void USBDevIntHandler(uint8_t bDevStatus){

}

void GPIO_isr(){
	uint8_t sig = 'A';
	if((IO0IntStatR & 1<<USB_TEST_IN)){
		IO0IntClr = 1<<USB_TEST_IN;
		switch(mode){
		case CTRL_REQ:
			USBHwEPWrite(CTRL_IN_EP, &sig, 1);
			break;
		case INTR_REQ:
			USBHwEPWrite(INTR_IN_EP, &sig, 1);
			break;
		case BULK_REQ:
			USBHwEPWrite(BULK_IN_EP, &sig, 1);
			break;
		case ISOC_REQ:
			USBHwEPWrite(ISOC_IN_EP, &sig, 1);
			break;
		default:
			break;
		}
		color_led_flash(1, BLUE_LED, FLASH_FAST);
	}
	VICAddress = 0x0;
}

void GPIO_init(){

	PINMODE1  |= 0x3<<0;          //pulldown on p0.16
	FIO0DIR   |= 1<<USB_TEST_OUT;
	IO0IntEnR |= 1<<USB_TEST_IN;
//	IO0IntEnF |= 1<<USB_TEST_IN;

    VICVectAddr17 = (unsigned int) GPIO_isr;
    ENABLE_GPIO_INT;
}

static void stream_task() {
	while(1){
	}
}

/*************************************************************************
  main
  ====
 **************************************************************************/

int main(void){


    pllstart_seventytwomhz();
    mam_enable();
    uart0_init_115200();
    init_color_led();

    uart0_putstring("\n\n\n***Starting IMU test (GFE2368)***\n\n");
    DBG(UART0,"Initializing USB stack...\n");

    // Initialize stack
    USBInit();

    // register descriptors
    USBRegisterDescriptors(abDescriptors);


    // register transfer handlers
    // register vender request handler for control transfers
    USBRegisterRequestHandler(REQTYPE_TYPE_VENDOR, HandleVendorRequest, abClassReqData);
    // register endpoint handlers
    USBHwRegisterEPIntHandler(INTR_IN_EP,  InterruptIn);
    USBHwRegisterEPIntHandler(INTR_OUT_EP, InterruptOut);
    USBHwRegisterEPIntHandler(BULK_IN_EP,  BulkIn);
    USBHwRegisterEPIntHandler(BULK_OUT_EP, BulkOut);
    USBHwRegisterEPIntHandler(ISOC_IN_EP,  IsochronousIn);
    USBHwRegisterEPIntHandler(ISOC_OUT_EP, IsochronousOut);
    //enable interrupt on nak for all endpoints todo:
    //USBHwNakIntEnable(0xFF);

    // register frame handler
    USBHwRegisterFrameHandler(USBFrameHandler);

    // register device event handler
    USBHwRegisterDevIntHandler(USBDevIntHandler);

    DBG(UART0,"Starting USB communication\n");

    VICVectPriority22 = 0x01;
    VICVectAddr22     = (unsigned int) USBIntHandler;

    // set up USB interrupt
    VICIntSelect      &= ~(1<<22);     // select IRQ for USB
    VICIntEnable      |= (1<<22);

    vic_enableIRQ();

    // connect to bus
    USBHwConnect(TRUE);

    cycle_led() ;
    color_led_flash(5, BLUE_LED, FLASH_FAST ) ;
    BLUE_LED_ON;

    DBG(UART0,"USBHwConnect\n");


    init_color_led();
    RED_LED_ON;

    GPIO_init();

    DBG(UART0, "stream_task()\n");

    stream_task();

    return 0;
}

