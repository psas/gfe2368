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
#include "L3G4200D.h"

i2c_iface g_i2c_channel;

void g_nullcallback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
	return;
}


void g_xact_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s) {
    uint16_t i;

    caller->i2c_ext_slave_address = i2c_s->i2c_ext_slave_address;
    caller->xact_success          = i2c_s->xact_success;
    caller->write_length          = i2c_s->write_length;
    caller->read_length           = i2c_s->read_length;
    caller->xact_active           = i2c_s->xact_active;

    for(i=0; i<I2C_MAX_BUFFER; ++i) {
        caller->i2c_tx_buffer[i] = i2c_s->i2c_tx_buffer[i];
        caller->i2c_rd_buffer[i] = i2c_s->i2c_rd_buffer[i];
    }
 }

void g_poll_wait(i2c_iface i2c_ch) {
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


int gyro_data_rdy(){

	i2c_master_xact_t       xact0_s;
	xact0_s.i2c_tx_buffer[0]  = i2c_create_write_address(L3G4200D_ADDR);
	xact0_s.i2c_tx_buffer[1]  = G_STATUS_REG; //data status register register addr
	xact0_s.write_length      = 0x2;
	xact0_s.i2c_tx_buffer[2] =  i2c_create_read_address(L3G4200D_ADDR);
	xact0_s.read_length       = 0x1;
	xact0_s.xact_active       = 0x1;
	xact0_s.xact_success      = 0x0;

	start_i2c_master_xact(g_i2c_channel, &xact0_s, &g_xact_callback);
	g_poll_wait(g_i2c_channel);
	return (xact0_s.i2c_rd_buffer[0] & G_ZYXDA) == G_ZYXDA;
}


void L3G4200D_get_data(XACT_FnCallback* xact_callback){
	i2c_master_xact_t       gyrodata;
	gyrodata.i2c_tx_buffer[0]  = i2c_create_write_address(L3G4200D_ADDR);
	gyrodata.i2c_tx_buffer[1]  = G_OUT_TEMP | G_AUTO_INCREMENT; //Auto increment through, temperature, data status,
	gyrodata.write_length      = 0x2;					   //then all the axis data registers
	gyrodata.i2c_tx_buffer[2]  = i2c_create_read_address(L3G4200D_ADDR);
	gyrodata.read_length       = 0x8;
	gyrodata.xact_active       = 0x1;
	gyrodata.xact_success      = 0x0;

	start_i2c_master_xact(g_i2c_channel, &gyrodata, g_xact_callback);
}

void L3G4200D_init(i2c_iface i2c_ch){
	i2c_master_xact_t       gyroinit;

	g_i2c_channel = i2c_ch;

	gyroinit.i2c_tx_buffer[0]  = i2c_create_write_address(L3G4200D_ADDR);
    gyroinit.i2c_tx_buffer[1]  = G_CTRL_REG1 | G_AUTO_INCREMENT;
    gyroinit.i2c_tx_buffer[2]  = G_ODR_100 | G_PWR | G_ALL_AXIS_ENABLE; //CTRL_REG1, turn on, data rate 100Hz
    gyroinit.i2c_tx_buffer[3]  = G_CTRL_REG2_DEFAULT;
    gyroinit.i2c_tx_buffer[4]  = G_INT2_DRDY;     //CTRL_REG3, interrupt 2 fires on data ready
    gyroinit.write_length      = 0x5;           //TODO: enable block data update
    gyroinit.read_length       = 0x0;
    gyroinit.xact_active       = 0x1;

    start_i2c_master_xact(g_i2c_channel, &gyroinit, &g_nullcallback);
}

