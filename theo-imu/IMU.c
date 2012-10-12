
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

#include "IMU.h"
#include "L3G4200D.h"
#include "LIS331HH.h"
#include "LSM303DLH.h"
#include "imu-device-host-interface.h"


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
    DBG(UART0, "**BulkOut read %d chars\n", iLen);
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
int VCOM_putchar(char c)
{
    return fifo_put(&txfifo, c) ? c : EOF;
}

/*
 * VCOM_putword
 * Writes one word to VCOM port
 */
//todo:why does disabling and re-enabling irqs break usb? also why does this func do it?
int VCOM_putword(int c) {
    int ret = 0;

//    vic_disableIRQ();
//    vic_disableFIQ();

    ret = fifo_putword(&txfifo,  c );
    if(!ret) {
//    	DBG(UART0, "fifo_putword fail\n");
    }

//    vic_enableIRQ();
//    vic_enableFIQ();

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

	imuPacket pkt;
	pkt.ID = ADDR_ACC;
	pkt.timestamp = LIS331HH_timestamp;
	pkt.status = i2c_s->i2c_rd_buffer[0];
	pkt.x = (uint16_t)i2c_s->i2c_rd_buffer[1] | (uint16_t)i2c_s->i2c_rd_buffer[2] << 8;
	pkt.y = (uint16_t)i2c_s->i2c_rd_buffer[3] | (uint16_t)i2c_s->i2c_rd_buffer[4] << 8;
	pkt.z = (uint16_t)i2c_s->i2c_rd_buffer[5] | (uint16_t)i2c_s->i2c_rd_buffer[6] << 8;
	pkt.extra_data = 0;

	submit_imu_packet(&pkt, VCOM_putchar);
}

void L3G4200D_get_data_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
	//TODO: send xact_success failure over USB

	imuPacket pkt;
	pkt.ID = ADDR_GYR;
	pkt.timestamp = L3G4200D_timestamp;
	pkt.status = i2c_s->i2c_rd_buffer[1];
	pkt.x = (uint16_t)i2c_s->i2c_rd_buffer[2] | (uint16_t)i2c_s->i2c_rd_buffer[3] << 8;
	pkt.y = (uint16_t)i2c_s->i2c_rd_buffer[4] | (uint16_t)i2c_s->i2c_rd_buffer[5] << 8;
	pkt.z = (uint16_t)i2c_s->i2c_rd_buffer[6] | (uint16_t)i2c_s->i2c_rd_buffer[7] << 8;
	pkt.extra_data = i2c_s->i2c_rd_buffer[0];//temperature data

	submit_imu_packet(&pkt, VCOM_putchar);
}

void LSM303DLH_m_get_data_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
	//TODO: send xact_success failure over USB

	imuPacket pkt;
	pkt.ID = ADDR_MAG;
	pkt.timestamp = LSM303DLH_m_timestamp;
	pkt.status = 0;
	pkt.x = (uint16_t)i2c_s->i2c_rd_buffer[0] | (uint16_t)i2c_s->i2c_rd_buffer[1] << 8;
	pkt.y = (uint16_t)i2c_s->i2c_rd_buffer[2] | (uint16_t)i2c_s->i2c_rd_buffer[3] << 8;
	pkt.z = (uint16_t)i2c_s->i2c_rd_buffer[4] | (uint16_t)i2c_s->i2c_rd_buffer[5] << 8;
	pkt.extra_data = 0;

	submit_imu_packet(&pkt, VCOM_putchar);
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

	L3G4200D_init(I2C0);
	LIS331HH_init(I2C1);
	LSM303DLH_init_m(I2C2);

	poll_wait(I2C0);
	poll_wait(I2C1);
	poll_wait(I2C2);

    timer_init(TIMER_0, 0x0 , CCLK_DIV1);
	RESET_TIMER0;
	START_TIMER0;

	//GPIO interrupt
    VICVectAddr17 = (unsigned int) IMU_isr;
    ENABLE_GPIO_INT;
}

/*
 * stream_task
 */
static void stream_task() {
	int c       = 0;
	const uint8_t LIS331HH_speed[] = {
			A_PWR_DOWN,
			A_LPWR_ODR_05 | A_ALL_AXIS_ENABLE,
			A_LPWR_ODR_1  | A_ALL_AXIS_ENABLE,
			A_LPWR_ODR_2  | A_ALL_AXIS_ENABLE,
			A_LPWR_ODR_5  | A_ALL_AXIS_ENABLE,
			A_LPWR_ODR_10 | A_ALL_AXIS_ENABLE,
			A_ODR_50      | A_ALL_AXIS_ENABLE,
			A_ODR_100	  | A_ALL_AXIS_ENABLE,
			A_ODR_400	  | A_ALL_AXIS_ENABLE,
			A_ODR_1000	  | A_ALL_AXIS_ENABLE};
	const int LIS331HH_Hz[] = {0, -5, 1, 2, 5, 10, 50, 100, 400, 1000};
	const int LIS331HH_Hz_max_index = sizeof(LIS331HH_Hz)/sizeof(int);
	int LIS331HH_cur_Hz = 5;	//A_LPWR_ODR_5, default speed

	const uint8_t L3G4200D_speed[] = {
			G_PWR_DOWN,
			G_ODR_100 | G_PWR | G_ALL_AXIS_ENABLE | G_BW_H,
			G_ODR_200 | G_PWR | G_ALL_AXIS_ENABLE | G_BW_H,
			G_ODR_400 | G_PWR | G_ALL_AXIS_ENABLE | G_BW_H,
			G_ODR_800 | G_PWR | G_ALL_AXIS_ENABLE | G_BW_H};
	const int L3G4200D_Hz[] = {0, 100, 200, 400, 800};
	const int L3G4200D_Hz_max_index = sizeof(L3G4200D_Hz)/sizeof(int);
	int L3G4200D_cur_Hz = 1; //G_ODR_100, default speed

	const uint8_t LSM303DLH_speed[] = {
			C_SLEEP    | C_MR_REG_M_MASK,
			C_ODR_0_75 | C_CRA_REG_M_MASK,
			C_ODR_1_5  | C_CRA_REG_M_MASK,
			C_ODR_3    | C_CRA_REG_M_MASK,
			C_ODR_7_5  | C_CRA_REG_M_MASK,
			C_ODR_15   | C_CRA_REG_M_MASK,
			C_ODR_30   | C_CRA_REG_M_MASK,
			C_ODR_75   | C_CRA_REG_M_MASK};
	const int LSM303DLH_Hz[] = {0, -75, 1, 3, 7, 15, 30, 75};
	const int LSM303DLH_Hz_max_index = sizeof(LSM303DLH_Hz)/sizeof(int);
	int LSM303DLH_cur_Hz = 4; //C_ODR_7_5, default speed

	runstate_g.state = STOP;

	while (1) {
		switch(runstate_g.state) {
		case GO:
			break;
		case STOP:
			// stop getting samples
//			IO0IntEnR &= ~(MAG_INT1 | MAG_INT1);
			IO0IntEnR &= ~(MAG_DRDY);
			IO0IntEnR &= ~(ACCEL_INT1);
			IO0IntEnR &= ~(GYRO_INT2);
			all_led_off();
			BLUE_LED_ON;
			break;
		case RESET:
			runstate_g.state = GO;
 			VCOM_init();
//			IO0IntEnR |= MAG_INT1 | MAG_INT1;
			IO0IntEnR |= MAG_DRDY;
			IO0IntEnR |= ACCEL_INT1;
			IO0IntEnR |= GYRO_INT2;
			if(L3G4200D_STUCK){
				L3G4200D_get_data(empty_callback);
			}
			if(LIS331HH_STUCK){
				LIS331HH_get_data(empty_callback);
			}
			if(LSM303DLH_M_STUCK){
				LSM303DLH_m_get_data(empty_callback);
			}
			break;
		default:
			DBG(UART0, "stream_task(): INVALID STATE\n");
			all_led_off();
			RED_LED_ON;
			break;
		}

		c = VCOM_getchar();
		switch(IMU_INST(c)){
		case EOF:
			break;
		case INST_GO:
			all_led_off();
			GREEN_LED_ON;
			runstate_g.state = GO;
			DBG(UART0,"Go detected\n");
			break;
		case INST_STOP:
			runstate_g.state = STOP;
			DBG(UART0,"Stop detected\n");
			break;
		case INST_RESET:
			runstate_g.state = RESET;
			DBG(UART0,"Reset detected\n");
			break;
		case INST_INC_SPEED:
			if((IMU_ADDR(c) & ADDR_ACC) && (LIS331HH_cur_Hz < LIS331HH_Hz_max_index)){
				++LIS331HH_cur_Hz;
				DBG(UART0, "Increased acc speed to %d Hz\n", LIS331HH_Hz[LIS331HH_cur_Hz]);
				LIS331HH_set_ctrl_reg(1, LIS331HH_speed[LIS331HH_cur_Hz]);
			}
			if((IMU_ADDR(c) & ADDR_GYR) && (L3G4200D_cur_Hz < L3G4200D_Hz_max_index)){
				++L3G4200D_cur_Hz;
				DBG(UART0, "Increased gyr speed to %d Hz\n", L3G4200D_Hz[L3G4200D_cur_Hz]);
				L3G4200D_set_ctrl_reg(1, L3G4200D_speed[L3G4200D_cur_Hz]);
			}
			if((IMU_ADDR(c) & ADDR_MAG) && (LSM303DLH_cur_Hz < LSM303DLH_Hz_max_index)){
				++LSM303DLH_cur_Hz;
				DBG(UART0, "Increased mag speed to %d Hz\n", LSM303DLH_Hz[LSM303DLH_cur_Hz]);
				LSM303DLH_m_set_ctrl_reg(1, LSM303DLH_speed[LSM303DLH_cur_Hz]);
				if(LSM303DLH_cur_Hz == 1) //waking up
					LSM303DLH_m_set_ctrl_reg(3, C_CONT_CONV & C_MR_REG_M_MASK);
			}
			break;
		case INST_DEC_SPEED:
			if((IMU_ADDR(c) & ADDR_ACC) && (LIS331HH_cur_Hz > 0)){
				--LIS331HH_cur_Hz;
				DBG(UART0, "Decreased acc speed to %d Hz\n", LIS331HH_Hz[LIS331HH_cur_Hz]);
				LIS331HH_set_ctrl_reg(1, LIS331HH_speed[LIS331HH_cur_Hz]);
			}
			if((IMU_ADDR(c) & ADDR_GYR) && (L3G4200D_cur_Hz > 0)){
				--L3G4200D_cur_Hz;
				DBG(UART0, "Decreased gyr speed to %d Hz\n", L3G4300D_Hz[L3G4200D_cur_Hz]);
				L3G4200D_set_ctrl_reg(1, L3G4200D_speed[L3G4200D_cur_Hz]);
			}
			if((IMU_ADDR(c) & ADDR_MAG) && (LSM303DLH_cur_Hz > 0)){
				--LSM303DLH_cur_Hz;
				DBG(UART0, "Decreased mag speed to %d Hz\n", LSM303DLH_Hz[LSM303DLH_cur_Hz]);
				if(LSM303DLH_cur_Hz == 0) //going to sleep
					LSM303DLH_m_set_ctrl_reg(3, C_SLEEP & C_MR_REG_M_MASK);
				else
					LSM303DLH_m_set_ctrl_reg(1, LSM303DLH_speed[LSM303DLH_cur_Hz]);
			}
			break;
		default:
			color_led_flash(5, RED_LED, FLASH_FAST );
			break;
		}
	}
}

/*************************************************************************
  main
  ====
 **************************************************************************/

int main(void){

	volatile uint32_t* udcaHeadArray[32] __attribute__((aligned(128))); //todo: does this work with linker
    pllstart_seventytwomhz();
    mam_enable();
    uart0_init_115200();
    init_color_led();

    uart0_putstring("\n\n\n***Starting IMU test (GFE2368)***\n\n");
    DBG(UART0,"Initializing USB stack...\n");

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

    // turn on USB DMA
    USBInitializeUSBDMA(udcaHeadArray);
    //todo:finish DMA

    // Initialize VCOM
    VCOM_init();

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

    DBG(UART0, "IMU_init()\n");
	IMU_init();

	DBG(UART0, "stream_task()\n");
    stream_task();

    return 0;
}

