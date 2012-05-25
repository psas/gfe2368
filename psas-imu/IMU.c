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
#include "printf-lpc.h"

#include "gfe2368-util.h"
#include "IMU.h"
#include "L3G4200D.h"
#include "LIS331HH.h"
#include "LSM303DLH.h"


axis_data 				xyz_data;
axis_data				gyro_data;
axis_data				mag_data;
uint8_t					temp_data;

void poll_wait(i2c_iface i2c_ch) {
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
	temp_data = (uint8_t)i2c_s->i2c_rd_buffer[0];
	gyro_data.x = ((int8_t)i2c_s->i2c_rd_buffer[3] << 8) + (int8_t)i2c_s->i2c_rd_buffer[2];
	gyro_data.y = ((int8_t)i2c_s->i2c_rd_buffer[5] << 8) + (int8_t)i2c_s->i2c_rd_buffer[4];
	gyro_data.z = ((int8_t)i2c_s->i2c_rd_buffer[7] << 8) + (int8_t)i2c_s->i2c_rd_buffer[6];
	gyro_data.modified = 1;
	if(!(i2c_s->xact_success)){
		uart0_putstring("\n***L3G4200D GET DATA FAILED***\n");
	}
}

void LIS331HH_get_data_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
	//axis data has been read, move it to axis data struct
	xyz_data.x = ((int8_t)i2c_s->i2c_rd_buffer[1] << 8) + (int8_t)i2c_s->i2c_rd_buffer[0];
	xyz_data.y = ((int8_t)i2c_s->i2c_rd_buffer[3] << 8) + (int8_t)i2c_s->i2c_rd_buffer[2];
	xyz_data.z = ((int8_t)i2c_s->i2c_rd_buffer[5] << 8) + (int8_t)i2c_s->i2c_rd_buffer[4];
	xyz_data.modified = 1;
	if(!(i2c_s->xact_success)){
		uart0_putstring("\n***LIS331HH GET DATA FAILED***\n");
	}
}

void LSM303DLH_m_get_data_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
	mag_data.x = ((int8_t)i2c_s->i2c_rd_buffer[0] << 8) + (int8_t)i2c_s->i2c_rd_buffer[1];
	mag_data.y = ((int8_t)i2c_s->i2c_rd_buffer[2] << 8) + (int8_t)i2c_s->i2c_rd_buffer[3];
	mag_data.z = ((int8_t)i2c_s->i2c_rd_buffer[4] << 8) + (int8_t)i2c_s->i2c_rd_buffer[5];
	mag_data.modified = 1;

	if(!(i2c_s->xact_success)){
		uart0_putstring("\n***LSM303DLH GET DATA FAILED***\n");
	}
}

void IMU_isr(){
	DISABLE_GPIO_INT;
	//CLR_SW_GPIO_INT;
	uart0_putstring("\n***ISR***\n");
	//check IOIntStatus bit 0 - LPCUM 10.5.6.1. If 1, PORT0 interrupt.
	if(!(IOIntStat & 1)){
		ENABLE_GPIO_INT;
		return; //interrupt is not for IMU
	}

	//IO0IntStatR for pin interrupt status
	//IO0IntClr to clear interrupt
	if((IO0IntStatR & (ACCEL_INT1 | ACCEL_INT2))){
		LIS331HH_get_data(LIS331HH_get_data_callback);
		IO0IntClr =	ACCEL_INT1 | ACCEL_INT2;
	}
//	if((IO0IntStatR & (MAG_INT1 | MAG_INT2 | MAG_DRDY)) == 1){
//		LSM303DLH_m_get_data(LSM303DLH_m_get_data_callback);
//		IO0IntClr =	MAG_INT1 | MAG_INT2 | MAG_DRDY;
//	}
//	if((IO0IntStatR & MAG_DRDY)){
//		LSM303DLH_m_get_data(LSM303DLH_m_get_data_callback);
//		IO0IntClr =	 MAG_DRDY;
//	}
//	if((IO0IntStatR & (GYRO_INT1 | GYRO_INT2)) == 1){
//		L3G4200D_get_data(L3G4200D_get_data_callback);
//		IO0IntClr =	GYRO_INT1 | GYRO_INT2;
//	}

	ENABLE_GPIO_INT; //enable in callback? I think it will be safe here as it is rising edge only
	VICAddress = 0x0;

}

//configures the uc for IMU work
void IMU_init(){

	//GPIO pinsel by default
	//pinmode?
	//GPIO input  by default
	//configure specific pins for interrupt. IO0IntEnR for rising edge interrupt
	//IO2IntEnR = ACCEL_INT1 | ACCEL_INT2 | MAG_INT1 | MAG_INT1 | MAG_DRDY | GYRO_INT1 | GYRO_INT2;
	//IO2IntEnR = MAG_DRDY;
//	IO0IntEnR = ACCEL_INT1 | ACCEL_INT2;
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

//    VICVectAddr17 = (unsigned int) IMU_isr; //uint?
//    VICAddress = 0x0;
//    ENABLE_GPIO_INT;
}

void IMU_task(){
	IMU_init();
	poll_wait(I2C0);
	poll_wait(I2C1);
	poll_wait(I2C2);

	uart0_putstring("\n***init done***\n");
	RED_LED_OFF;
	BLUE_LED_ON;
	mag_data.modified = 0;
	gyro_data.modified = 0;
	xyz_data.modified = 0;

	//if any of the int lines are high sw trigger interrupt.?
//	if((FIO0PIN & MAG_DRDY)){
//		RAISE_GPIO_INT;
//	}
	//printf_lpc(UART0, "    GPIO Status: 0x%X\n", (FIO0PIN & MAG_DRDY));
	while(1){
		//if(mag_data.modified == 1){
		//uart0_putstring("***loop***\n");
		if(accel_data_rdy()){
//		if(xyz_data.modified == 1){
			LIS331HH_get_data(LIS331HH_get_data_callback);
			xyz_data.modified = 0;
			color_led_flash(1, RED_LED, FLASH_FAST );
			printf_lpc(UART0, "    X axis: %d     y axis: %d    z axis: %d\n",
					   xyz_data.x,
					   xyz_data.y,
					   xyz_data.z);
		}
//		if(gyro_data_rdy()){
//			L3G4200D_get_data(L3G4200D_get_data_callback);
//			gyro_data.modified = 0;
//			poll_wait(I2C0);
//			printf_lpc(UART0, "    X gyro: %d     y gyro: %d    z gyro: %d, TEMP: %d\n",
//					   gyro_data.x,
//					   gyro_data.y,
//					   gyro_data.z,
//					   temp_data);

//		}
//		if(mag_data_rdy_m()){
//			LSM303DLH_m_get_data(LSM303DLH_m_get_data_callback);
//			mag_data.modified = 0;
//			color_led_flash(1, RED_LED, FLASH_FAST );
//			printf_lpc(UART0, "     X mag: %d     y mag: %d    z mag: %d\n",
//					   mag_data.x,
//					   mag_data.y,
//					   mag_data.z);
//		}
	}
}


int main (void) {
    pllstart_sixtymhz() ;
    uart0_init_115200() ;
    vic_enableIRQ();
    vic_enableFIQ();

    i2c_init(I2C0, DEFAULT);
    i2c_freq(I2C0, (uint16_t) 100, (uint16_t) 100);//this should be the normal i2c freq...?

    i2c_init(I2C1, I2C1_ALTPIN);
    i2c_freq(I2C1, (uint16_t) 100, (uint16_t) 100);

    i2c_init(I2C2, DEFAULT);
    i2c_freq(I2C2, (uint16_t) 100, (uint16_t) 100);

    uart0_putstring("\n***Starting IMU test (GFE2368)***\n\n");
    init_color_led();
    RED_LED_ON;

    IMU_task();

    uart0_putstring("\n\n***Done. Also this shouldn't print***\n\n");

    return(0);
}


//PINSEL1  |= 0x1 << 22; //enables EINT1 on p2.11
//PINMODE1 |= 0x2 << 22; //make p2.11 pushpull
//EXTMODE = 0x1 << 1; //make eint1 edge detect
