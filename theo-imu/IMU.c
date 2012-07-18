
/*
 * datapath-test.c
 *
 * Test the datapath from a sensor to
 * the LPC to USB to the FC and from the FC
 * to the LPC.
 * 
 */

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

#include "usbapi.h"

#include "serial-fifo.h"

#include "gfe2368-util.h"

//#include "datapath-test.h"
#include "IMU.h"
#include "L3G4200D.h"
#include "LIS331HH.h"
#include "LSM303DLH.h"


#include "lpc23xx-binsem.h"

#define BAUD_RATE               115200

#define INT_IN_EP               0x81
#define BULK_OUT_EP             0x05
#define BULK_IN_EP              0x82

#define MAX_PACKET_SIZE 64

#define LE_WORD(x)              ((x)&0xFF),((x)>>8)

// CDC definitions
#define CS_INTERFACE            0x24
#define CS_ENDPOINT             0x25

#define SET_LINE_CODING         0x20
#define GET_LINE_CODING         0x21
#define SET_CONTROL_LINE_STATE  0x22

static TLineCoding      LineCoding = {115200, 0, 0, 8};
static uint8_t          abBulkBuf[64];
static uint8_t          abClassReqData[8];
static volatile BOOL    fBulkInBusy;
static volatile BOOL    fChainDone;

static fifo_type           txfifo;
static fifo_type           rxfifo;

struct {
   runstate_type state;
} runstate_g;

#define ACCEL	1
#define GYRO	2
#define MAG		3

static uint32_t L3G4200D_timestamp;
static uint32_t LIS331HH_timestamp;
static uint32_t LSM303DLH_m_timestamp;


static const uint8_t abDescriptors[] = {

    // device descriptor
    0x12,
    DESC_DEVICE,
    LE_WORD(0x0101),                    // bcdUSB
    0x02,                               // bDeviceClass
    0x00,                               // bDeviceSubClass
    0x00,                               // bDeviceProtocol
    MAX_PACKET_SIZE0,                   // bMaxPacketSize
    LE_WORD(0xFFFF),                    // idVendor
    LE_WORD(0x0005),                    // idProduct
    LE_WORD(0x0100),                    // bcdDevice
    0x01,                               // iManufacturer
    0x02,                               // iProduct
    0x03,                               // iSerialNumber
    0x01,                               // bNumConfigurations

    // configuration descriptor
    0x09,
    DESC_CONFIGURATION,
    LE_WORD(67),                        // wTotalLength
    0x02,                               // bNumInterfaces
    0x01,                               // bConfigurationValue
    0x00,                               // iConfiguration
    0xC0,                               // bmAttributes
    0x32,                               // bMaxPower
    // control class interface
    0x09,
    DESC_INTERFACE,
    0x00,                               // bInterfaceNumber
    0x00,                               // bAlternateSetting
    0x01,                               // bNumEndPoints
    0x02,                               // bInterfaceClass
    0x02,                               // bInterfaceSubClass
    0x01,                               // bInterfaceProtocol, linux requires value of 1 for the cdc_acm module
    0x00,                               // iInterface
    // header functional descriptor
    0x05,
    CS_INTERFACE,
    0x00,
    LE_WORD(0x0110),
    // call management functional descriptor
    0x05,
    CS_INTERFACE,
    0x01,
    0x01,                                // bmCapabilities = device handles call management
    0x01,                                // bDataInterface
    // ACM functional descriptor
    0x04,
    CS_INTERFACE,
    0x02,
    0x02,                                // bmCapabilities
    // union functional descriptor
    0x05,
    CS_INTERFACE,
    0x06,
    0x00,                                // bMasterInterface
    0x01,                                // bSlaveInterface0
    // notification EP
    0x07,
    DESC_ENDPOINT,
    INT_IN_EP,                           // bEndpointAddress
    0x03,                                // bmAttributes = intr
    LE_WORD(8),                          // wMaxPacketSize
    0x0A,                                // bInterval
    // data class interface descriptor
    0x09,
    DESC_INTERFACE,
    0x01,                                // bInterfaceNumber
    0x00,                                // bAlternateSetting
    0x02,                                // bNumEndPoints
    0x0A,                                // bInterfaceClass = data
    0x00,                                // bInterfaceSubClass
    0x00,                                // bInterfaceProtocol
    0x00,                                // iInterface
    // data EP OUT
    0x07,
    DESC_ENDPOINT,
    BULK_OUT_EP,                         // bEndpointAddress
    0x02,                                // bmAttributes = bulk
    LE_WORD(MAX_PACKET_SIZE),            // wMaxPacketSize
    0x00,                                // bInterval
    // data EP in
    0x07,
    DESC_ENDPOINT,
    BULK_IN_EP,                          // bEndpointAddress
    0x02,                                // bmAttributes = bulk
    LE_WORD(MAX_PACKET_SIZE),            // wMaxPacketSize
    0x00,                                // bInterval

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
//
//static axis_data 				accel_data;
//static axis_data				gyro_data;
//static axis_data				mag_data;
//static uint8_t					temp_data;

/**
  Local function to handle incoming bulk data

  @param [in] bEP
  @param [in] bEPStatus
  */
static void BulkOut(uint8_t bEP, uint8_t bEPStatus) {
    int i, iLen;

    if (fifo_free(&rxfifo) < MAX_PACKET_SIZE) {
        // may not fit into fifo
        return;
    }

    // get data from USB into intermediate buffer
    iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));
    for (i = 0; i < iLen; i++) {
        // put into FIFO
        if (!fifo_put(&rxfifo, abBulkBuf[i])) {
            // overflow... :(
            ASSERT(FALSE);
            break;
        }
    }
}


/**
  Sends the next packet in chain of packets to the host

  @param [in] bEP
  @param [in] bEPStatus
  */
static void SendNextBulkIn(uint8_t bEP, BOOL fFirstPacket)
{
    int iLen;

    // this transfer is done
    fBulkInBusy = FALSE;

    // first packet?
    if (fFirstPacket) {
        fChainDone = FALSE;
    }

    // last packet?
    if (fChainDone) {
        return;
    }

    // get up to MAX_PACKET_SIZE bytes from transmit FIFO into intermediate buffer
    for (iLen = 0; iLen < MAX_PACKET_SIZE; iLen++) {
        if (!fifo_get(&txfifo, &abBulkBuf[iLen])) {
            break;
        }
    }

    // send over USB
    USBHwEPWrite(bEP, abBulkBuf, iLen);
    fBulkInBusy = TRUE;

    // was this a short packet?
    if (iLen < MAX_PACKET_SIZE) {
        fChainDone = TRUE;
    }
}


/**
  Local function to handle outgoing bulk data

  @param [in] bEP
  @param [in] bEPStatus
  */
static void BulkIn(uint8_t bEP, uint8_t bEPStatus)
{
    SendNextBulkIn(bEP, FALSE);
}


/**
  Local function to handle the USB-CDC class requests

  @param [in] pSetup
  @param [out] piLen
  @param [out] ppbData
  */
static BOOL HandleClassRequest(TSetupPacket *pSetup, int *piLen, uint8_t **ppbData)
{
    switch (pSetup->bRequest) {

        // set line coding
        case SET_LINE_CODING:
            //DBG(UART0,"SET_LINE_CODING\n");
            memcpy((uint8_t *)&LineCoding, *ppbData, 7);
            *piLen = 7;
            DBG(UART0,"dwDTERate=%u, bCharFormat=%u, bParityType=%u, bDataBits=%u\n",
                    LineCoding.dwDTERate,
                    LineCoding.bCharFormat,
                    LineCoding.bParityType,
                    LineCoding.bDataBits);
            break;

            // get line coding
        case GET_LINE_CODING:
            //DBG(UART0,"GET_LINE_CODING\n");
            *ppbData = (uint8_t *)&LineCoding;
            *piLen = 7;
            break;

            // set control line state
        case SET_CONTROL_LINE_STATE:
            // bit0 = DTR, bit = RTS
            //DBG(UART0,"SET_CONTROL_LINE_STATE %X\n", pSetup->wValue);
            break;

        default:
            return FALSE;
    }
    return TRUE;
}


/**
  Initialises the VCOM port.
  Call this function before using VCOM_putchar or VCOM_getchar
  */
void VCOM_init(void)
{

    fifo_init(&txfifo);
    fifo_init(&rxfifo);
    fBulkInBusy = FALSE;
    fChainDone = TRUE;
}

/**
  Writes one character to VCOM port

  @param [in] c character to write
  @returns character written, or EOF if character could not be written
  */
int VCOM_putchar(int c)
{
    return fifo_put(&txfifo, c) ? c : EOF;
}

/*
 * VCOM_putword
 * Writes one word to VCOM port
 */
int VCOM_putword(int c) {
    int ret = 0;

    vic_disableIRQ();
    vic_disableFIQ();

    ret = fifo_putword(&txfifo,  c );
    if(!ret) {
    	DBG(UART0, "fifo_putword fail\n");
    }

    vic_enableIRQ();
    vic_enableFIQ();

    return(0);
}

/**
  Reads one character from VCOM port

  @returns character read, or EOF if character could not be read
  */
int VCOM_getchar(void)
{
    uint8_t c;

    return fifo_get(&rxfifo, &c) ? c : EOF;
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
static void USBFrameHandler(uint16_t wFrame)
{
    if (!fBulkInBusy && (fifo_avail(&txfifo) != 0)) {
        // send first packet
        SendNextBulkIn(BULK_IN_EP, TRUE);
    }
}


/**
  USB device status handler

  Resets state machine when a USB reset is received.
  */
static void USBDevIntHandler(uint8_t bDevStatus)
{
    if ((bDevStatus & DEV_STATUS_RESET) != 0) {
        fBulkInBusy = FALSE;
    }
}

static void poll_wait(i2c_iface i2c_ch) {
	switch(i2c_ch){
    case I2C0:
    	while(is_binsem_locked(&i2c0_binsem_g)== 1);
    	break;
    case I2C1:
    	while(is_binsem_locked(&i2c1_binsem_g)== 1);
    	break;
    case I2C2:
    	while(is_binsem_locked(&i2c2_binsem_g)== 1);
    	break;
     }
}

void LIS331HH_get_data_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
	//TODO: send xact_success failure over USB
	if(!(i2c_s->xact_success)){
		uart0_putstring("\n***LIS331HH GET DATA FAILED***\n");
		return;
	}
	//TODO: Sensor ID
//	VCOM_putchar(ACCEL);
	//TODO: Time stamp
//	VCOM_putword(LIS331HH_timestamp);
	//TODO: Sensor status register
//	VCOM_putchar(i2c_s->i2c_rd_buffer[0]); //status register

	VCOM_putchar(i2c_s->i2c_rd_buffer[1]); //x low
	VCOM_putchar(i2c_s->i2c_rd_buffer[2]); //x high

	VCOM_putchar(i2c_s->i2c_rd_buffer[3]); //y low
	VCOM_putchar(i2c_s->i2c_rd_buffer[4]); //y high

	VCOM_putchar(i2c_s->i2c_rd_buffer[5]); //z low
	VCOM_putchar(i2c_s->i2c_rd_buffer[6]); //z high
}

void L3G4200D_get_data_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
	//TODO: send xact_success failure over USB
	//TODO: Sensor ID
//	VCOM_putchar(GYRO);
	//TODO: Time stamp
//	VCOM_putword(L3G4200D_timestamp);
	//TODO: Sensor status register
//	VCOM_putchar(i2c_s->i2c_rd_buffer[1]); //status register

	VCOM_putchar(i2c_s->i2c_rd_buffer[2]); //x low
	VCOM_putchar(i2c_s->i2c_rd_buffer[3]); //x high

	VCOM_putchar(i2c_s->i2c_rd_buffer[4]); //y low
	VCOM_putchar(i2c_s->i2c_rd_buffer[5]); //y high

	VCOM_putchar(i2c_s->i2c_rd_buffer[6]); //z low
	VCOM_putchar(i2c_s->i2c_rd_buffer[7]); //z high

//	VCOM_putchar(i2c_s->i2c_rd_buffer[0]); //temp
}

void LSM303DLH_m_get_data_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
	//TODO: send xact_success failure over USB
	//TODO: Sensor ID
//	VCOM_putchar(MAG);
	//TODO: Time stamp
//	VCOM_putword(LSM303DLH_m_timestamp);
	//TODO: Sensor status register?

	VCOM_putchar(i2c_s->i2c_rd_buffer[0]); //x low
	VCOM_putchar(i2c_s->i2c_rd_buffer[1]); //x high

	VCOM_putchar(i2c_s->i2c_rd_buffer[2]); //y low
	VCOM_putchar(i2c_s->i2c_rd_buffer[3]); //y high

	VCOM_putchar(i2c_s->i2c_rd_buffer[4]); //z low
	VCOM_putchar(i2c_s->i2c_rd_buffer[5]); //z high
}

static void empty_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s) {
	if(!(i2c_s->xact_success)){
		uart0_putstring("\n***Some IMU I2C call failed***\n");
	}
	return;
}

void IMU_isr(){
//	DISABLE_GPIO_INT;
	uint32_t timestamp = T0TC;
	//check IOIntStatus bit 0 - LPCUM 10.5.6.1. If 1, PORT0 interrupt.
	if(!(IOIntStat & 1)){
//		ENABLE_GPIO_INT;
		return; //interrupt is not for IMU
	}

	//IO0IntStatR for pin interrupt status
	//IO0IntClr to clear interrupt
	if((IO0IntStatR & GYRO_INT2) || L3G4200D_STUCK){
		L3G4200D_timestamp = timestamp;
		L3G4200D_get_data(L3G4200D_get_data_callback);
		IO0IntClr = GYRO_INT2;
	}

	if((IO0IntStatR & ACCEL_INT1) || LIS331HH_STUCK){
		LIS331HH_timestamp = timestamp;
		LIS331HH_get_data(LIS331HH_get_data_callback);
		IO0IntClr =	ACCEL_INT1;
	}

	if((IO0IntStatR & MAG_DRDY) || LSM303DLH_M_STUCK){
		LSM303DLH_m_timestamp = timestamp;
		LSM303DLH_m_get_data(LSM303DLH_m_get_data_callback);
		IO0IntClr =	 MAG_DRDY;
	}

//	ENABLE_GPIO_INT;
	VICAddress = 0x0;
}

void IMU_init(){
    i2c_init(I2C0, DEFAULT);
    i2c_kHz(I2C0, 400);

    i2c_init(I2C1, I2C1_ALTPIN);
    i2c_kHz(I2C1, 400);

    i2c_init(I2C2, DEFAULT);
    i2c_kHz(I2C2, 400);

	//configure other wires, (addresses, etc.)
	FIO0DIR |= ACCEL_SA0 | ACCEL_CS | MAG_SA | GYRO_SA0 | GYRO_CS;
	FIO0SET = ACCEL_CS | GYRO_CS;
	FIO0CLR = ACCEL_SA0 | MAG_SA | GYRO_SA0;

//	L3G4200D_init(I2C0);
	LIS331HH_init(I2C1);
//	LSM303DLH_init_m(I2C2);

	poll_wait(I2C0);
	poll_wait(I2C1);
	poll_wait(I2C2);

    timer_init(TIMER_0, 0x0 , CCLK_DIV1);
	RESET_TIMER0;
	START_TIMER0;

    VICVectAddr17 = (unsigned int) IMU_isr; //uint?
    ENABLE_GPIO_INT;
}

/*
 * stream_task
 */
static void stream_task() {
	int c       = 0;
	const uint8_t LIS331HH_speed[] = {0x5F, 0x7F, 0x9F, 0xBF, 0xDF, 0x27, 0x2F, 0x37, 0x3F}; //magic numbers
	const int LIS331HH_Hz[] = {5, 1, 2, 5, 10, 50, 100, 400, 1000};							 //casts correct spell
	int current_speed = 4;	//A_LPWR_ODR_5
	runstate_g.state = STOP;

	VCOM_putword(0xfeed);

	// do USB stuff in interrupt
	while (1) {
//		DBG(UART0, "State is: %u\n", (uint32_t) runstate_g.state);
//	        util_wait_msecs(20);
		switch(runstate_g.state) {
		case GO:
// 			clear fifo?
//			IO0IntEnR |= MAG_INT1 | MAG_INT1;
//			IO0IntEnR |= MAG_DRDY;
			IO0IntEnR |= ACCEL_INT1;
//			IO0IntEnR |= GYRO_INT2;
			if(L3G4200D_STUCK){
				L3G4200D_get_data(empty_callback);
			}
			if(LIS331HH_STUCK){
				LIS331HH_get_data(empty_callback);
			}
			if(LSM303DLH_M_STUCK){
				LSM303DLH_m_get_data(empty_callback);
			}

			all_led_off();
			GREEN_LED_ON;
			runstate_g.state = GO;
			break;
		case STOP:
			IO0IntClr = ACCEL_INT1;
//			IO0IntClr = GYRO_INT2;
//			IO0IntClr = MAG_DRDY;
			all_led_off();
			BLUE_LED_ON;
			// stop getting samples
			break;
		case RESET:
			runstate_g.state = GO;
			break;
		default:
			DBG(UART0, "stream_task(): INVALID STATE\n");
			all_led_off();
			RED_LED_ON;
			break;
		}

		c = VCOM_getchar();
		switch(c){
		case EOF:
			//show on console
			break;

		case 'g':
			all_led_off();
			GREEN_LED_ON;
			runstate_g.state = GO;
			DBG(UART0,"Go detected\n");
			break;

		case 's':
			runstate_g.state = STOP;
			DBG(UART0,"Stop detected\n");
			break;

		case 'm':
			printf_lpc(UART0,"\nOptions: (s)-stop, (g)-go, (q)-quit\n(+)-increase sample rate, (-)-decrease sample rate\n");
			DBG(UART0,"Menu detected\n");
			break;

		case 'q':
			runstate_g.state = STOP;
			DBG(UART0,"Quit detected\n")
			break;

		case 'r':
			runstate_g.state = RESET;
			DBG(UART0,"Quit detected\n");
			break;

		case '+':
			if(current_speed < 8){
				++current_speed;
				printf_lpc(UART0, "Increased speed to %d Hz\n", LIS331HH_Hz[current_speed]);
				LIS331HH_set_ctrl_reg(1, LIS331HH_speed[current_speed]);
			}
			break;

		case '-':
			if(current_speed > 0){
				--current_speed;
				printf_lpc(UART0, "Decreased speed to %d Hz\n", LIS331HH_Hz[current_speed]);
				LIS331HH_set_ctrl_reg(1, LIS331HH_speed[current_speed]);
			}
			break;

		default:
			if ((c == 9) || (c == 10) || (c == 13) || ((c >= 32) && (c <= 126))) {
				if(VCOM_putchar(c) == EOF){
					DBG(UART0, "Failed putting char in VCOM");
					runstate_g.state = RESET;
				}
				DBG(UART0,"%c", c);
			} else {
				color_led_flash(5, RED_LED, FLASH_FAST ) ;
				DBG(UART0,".");
			}
			break;

		}

		}
	}

}


/*************************************************************************
  main
  ====
 **************************************************************************/

int main(void){

    DBG(UART0,"\n***Start USB (bulk) datapath-test.***\n");
    FIO_ENABLE;

    pllstart_seventytwomhz() ;

    mam_enable();

    uart0_init_115200() ;

    init_color_led() ;

    DBG(UART0,"Initialising USB stack...\n");

    // Initialize stack
    USBInit();

    DBG(UART0,"Past USBInit\n");

    // register descriptors
    USBRegisterDescriptors(abDescriptors);

    DBG(UART0,"Past USBRegisterDescriptors\n");

    // register class request handler
    USBRegisterRequestHandler(REQTYPE_TYPE_CLASS, HandleClassRequest, abClassReqData);

    // register endpoint handlers
    USBHwRegisterEPIntHandler(INT_IN_EP, NULL);
    USBHwRegisterEPIntHandler(BULK_IN_EP, BulkIn);
    USBHwRegisterEPIntHandler(BULK_OUT_EP, BulkOut);

    // register frame handler
    USBHwRegisterFrameHandler(USBFrameHandler);

    // register device event handler
    USBHwRegisterDevIntHandler(USBDevIntHandler);

    // Initialize VCOM
    VCOM_init();

    DBG(UART0,"Starting USB communication\n");

    VICVectPriority22 = 0x01;
    VICVectAddr22     = (int) USBIntHandler;

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

    uart0_putstring("\n\n\n***Starting IMU test (GFE2368)***\n\n");
    init_color_led();
    RED_LED_ON;

	IMU_init();

    stream_task();

    return 0;
}

