/*
 * L3G4200D.c
 *
 *  Created on: Nov 21, 2011
 *      Author: theo
 */


#include <limits.h>
#include <stdint.h>

#include "lpc23xx.h"

#include "lpc23xx-pll.h"
#include "lpc23xx-binsem.h"
#include "lpc23xx-i2c.h"
#include "lpc23xx-uart.h"
#include "lpc23xx-util.h"
#include "lpc23xx-vic.h"
#include "lpc23xx-timer.h"
#include "printf-lpc.h"

#include "gfe2368-util.h"
#include "IMU.h"
#include "L3G4200D.h"
#include "LIS331HH.h"
#include "LSM303DLH.h"


static axis_data 				accel_data;
static axis_data				gyro_data;
static axis_data				mag_data;
static uint8_t					temp_data;

static uint32_t L3G4200D_timestamp;
static uint32_t LIS331HH_timestamp;
static uint32_t LSM303DLH_m_timestamp;

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

void L3G4200D_get_data_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
	//axis data has been read, move it to axis data struct
//	if(!(i2c_s->xact_success)){
//		uart0_putstring("\n***L3G4200D GET DATA FAILED***\n");
//		return;
//	}
//	if(!L3G4200D_data_available(i2c_s->i2c_rd_buffer[1])){
//		uart0_putstring("\n***L3G4200D retrieved old data***\n");
//		//todo: Determine old axis?
//		return;
//	}
	temp_data = i2c_s->i2c_rd_buffer[0];
	gyro_data.x = (int16_t)((uint16_t)i2c_s->i2c_rd_buffer[3] << 8) + (uint16_t)i2c_s->i2c_rd_buffer[2];
	gyro_data.y = (int16_t)((uint16_t)i2c_s->i2c_rd_buffer[5] << 8) + (uint16_t)i2c_s->i2c_rd_buffer[4];
	gyro_data.z = (int16_t)((uint16_t)i2c_s->i2c_rd_buffer[7] << 8) + (uint16_t)i2c_s->i2c_rd_buffer[6];

//	if(L3G4200D_data_overrun(i2c_s->i2c_rd_buffer[1])){
//		uart0_putstring("\n***L3G4200D data overrun***\n");
//		//todo: Determine old axis?
//	}
//	if(gyro_data.modified == TRUE){
//		uart0_putstring("\n***L3G4200D data not printed since last acquisition***\n");
//	}
	gyro_data.modified = TRUE;
}

void LIS331HH_get_data_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){

//	if(!(i2c_s->xact_success)){
//		uart0_putstring("\n***LIS331HH GET DATA FAILED***\n");
//		return;
//	}
//	if(!LIS331HH_data_available(i2c_s->i2c_rd_buffer[0])){
//		uart0_putstring("\n***LIS331HH retrieved old data***\n");
//		return;
//	}

//	uint32_t end = T0TC;
//	uint32_t tick_per_usec = microsecondsToCPUTicks(1) ;
//	printf_lpc(UART0, "time to complete sample: %d us\n", (end-timestamp)/tick_per_usec);


	accel_data.x = (int16_t)((uint16_t)i2c_s->i2c_rd_buffer[2] << 8) + (uint16_t)i2c_s->i2c_rd_buffer[1];
	accel_data.y = (int16_t)((uint16_t)i2c_s->i2c_rd_buffer[4] << 8) + (uint16_t)i2c_s->i2c_rd_buffer[3];
	accel_data.z = (int16_t)((uint16_t)i2c_s->i2c_rd_buffer[6] << 8) + (uint16_t)i2c_s->i2c_rd_buffer[5];
//
//	if(LIS331HH_data_overrun(i2c_s->i2c_rd_buffer[0])){
//		uart0_putstring("\n***LIS331HH data overrun***\n");
//	}
//	if(accel_data.modified == TRUE){
//		uart0_putstring("\n***LIS331HH data not printed since last acquisition***\n");
//	}
	accel_data.modified = TRUE;
}

void LSM303DLH_m_get_data_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
//	if(!(i2c_s->xact_success)){
//		uart0_putstring("\n***LSM303DLH_M GET DATA FAILED***\n");
//	}
	mag_data.x = (int16_t)((uint16_t)i2c_s->i2c_rd_buffer[0] << 8) + (uint16_t)i2c_s->i2c_rd_buffer[1];
	mag_data.y = (int16_t)((uint16_t)i2c_s->i2c_rd_buffer[2] << 8) + (uint16_t)i2c_s->i2c_rd_buffer[3];
	mag_data.z = (int16_t)((uint16_t)i2c_s->i2c_rd_buffer[4] << 8) + (uint16_t)i2c_s->i2c_rd_buffer[5];
//	if(mag_data.modified == TRUE){
//		uart0_putstring("\n***LSM303DLH_M data not printed since last acquisition***\n");
//	}
	mag_data.modified = TRUE;
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
	if((IO0IntStatR & GYRO_INT2)){// || L3G4200D_STUCK){
		L3G4200D_timestamp = timestamp;
		L3G4200D_get_data(L3G4200D_get_data_callback);
		IO0IntClr = GYRO_INT2;
	}

	if((IO0IntStatR & ACCEL_INT1)){// || LIS331HH_STUCK){
		LIS331HH_timestamp = timestamp;
		LIS331HH_get_data(LIS331HH_get_data_callback);
		IO0IntClr =	ACCEL_INT1;
	}

	if((IO0IntStatR & MAG_DRDY)){// || LSM303DLH_M_STUCK){
		LSM303DLH_m_timestamp = timestamp;
		LSM303DLH_m_get_data(LSM303DLH_m_get_data_callback);
		IO0IntClr =	 MAG_DRDY;
	}

//	ENABLE_GPIO_INT;
	VICAddress = 0x0;
}

//configures the uc for IMU work
void IMU_init(){

	//GPIO pinsel by default
	//pinmode?
	//GPIO input  by default
	//configure specific pins for interrupt. IO0IntEnR for rising edge interrupt
//	IO0IntEnR |= MAG_INT1 | MAG_INT1;
	IO0IntEnR |= MAG_DRDY;
	IO0IntEnR |= ACCEL_INT1;
	IO0IntEnR |= GYRO_INT2;
	//configure other wires, (addresses, etc.)
	//p0.9  = 1 (accel i2c mode)
	//p0.6  = 0 (accel i2c lsb 0)
	//P0.25 = 1 (gyro i2c mode)
	//P0.26 = 0 (gyro i2c lsb 0)
	//p0.21 = 0 (mag i2c lsb 0)
	//FIO gets enabled somewhere (PLL init stuff I think?) so use FIO
	FIO0DIR |= ACCEL_SA0 | ACCEL_CS | MAG_SA | GYRO_SA0 | GYRO_CS;
	FIO0SET = ACCEL_CS | GYRO_CS;
	FIO0CLR = ACCEL_SA0 | MAG_SA | GYRO_SA0;

	L3G4200D_init(I2C0);
	LIS331HH_init(I2C1);
	LSM303DLH_init_m(I2C2);

    timer_init(TIMER_0, 0x0 , CCLK_DIV1);
	RESET_TIMER0;
	START_TIMER0;

    VICVectAddr17 = (unsigned int) IMU_isr; //uint?
    ENABLE_GPIO_INT;
}

void IMU_task(){
	IMU_init();
	poll_wait(I2C0);
	poll_wait(I2C1);
	poll_wait(I2C2);

	uart0_putstring("\n***init done***\n");
	RED_LED_OFF;
	BLUE_LED_ON;
	mag_data.modified = FALSE;
	gyro_data.modified = FALSE;
	accel_data.modified = FALSE;

	//if any of the int lines are high, read data to clear.
	if(L3G4200D_STUCK){
		L3G4200D_get_data(empty_callback);
	}
	if(LIS331HH_STUCK){
		LIS331HH_get_data(empty_callback);
	}
	if(LSM303DLH_M_STUCK){
		LSM303DLH_m_get_data(empty_callback);
	}

	while(1){

		if(accel_data.modified == TRUE){
			printf_lpc(UART0, "    X axis: %d     y axis: %d    z axis: %d\n",
					   accel_data.x,
					   accel_data.y,
					   accel_data.z);
			accel_data.modified = FALSE;
		}

		if(gyro_data.modified){
			printf_lpc(UART0, "    X gyro: %d     y gyro: %d    z gyro: %d, TEMP: %d\n",
					   gyro_data.x,
					   gyro_data.y,
					   gyro_data.z,
					   temp_data);
			gyro_data.modified = FALSE;

		}
		if(mag_data.modified){
			printf_lpc(UART0, "    X mag: %d     y mag: %d    z mag: %d\n",
					   mag_data.x,
					   mag_data.y,
					   mag_data.z);
			mag_data.modified = FALSE;
		}
	}
}


int main (void) {
    pllstart_seventytwomhz() ;
    uart0_init_115200() ;
    vic_enableIRQ();
    vic_enableFIQ();

    i2c_init(I2C0, DEFAULT);
    i2c_kHz(I2C0, 400);

    i2c_init(I2C1, I2C1_ALTPIN);
    i2c_kHz(I2C1, 400);

    i2c_init(I2C2, DEFAULT);
    i2c_kHz(I2C2, 400);

    uart0_putstring("\n\n\n***Starting IMU test (GFE2368)***\n\n");
    init_color_led();
    RED_LED_ON;

    IMU_task();

    uart0_putstring("\n\n***Done. Also this shouldn't print***\n\n");

    return(0);
}
