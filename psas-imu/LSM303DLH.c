/*
 * LSM303DLH.c
 *
 *  Created on: Nov 22, 2011
 *      Author: theo
 */


#include <limits.h>
#include <stdint.h>

#include "lpc23xx.h"

#include "lpc23xx-binsem.h"
#include "lpc23xx-i2c.h"
#include "lpc23xx-uart.h"
#include "lpc23xx-util.h"
#include "lpc23xx-vic.h"
#include "printf-lpc.h"

#include "gfe2368-util.h"
#include "LSM303DLH.h"


i2c_iface c_i2c_channel;

void c_nullcallback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s){
	return;
}


void c_xact_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s) {
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

void c_poll_wait(i2c_iface i2c_ch) {
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


int mag_data_rdy_m(){

	i2c_master_xact_t       xact0_s;
	xact0_s.i2c_tx_buffer[0]  = i2c_create_write_address(LSM303DLH_ADDR_M);
	xact0_s.i2c_tx_buffer[1]  = C_SR_REG_M; //data status register register addr
	xact0_s.write_length      = 0x2;
	xact0_s.i2c_tx_buffer[2] =  i2c_create_read_address(LSM303DLH_ADDR_M);
	xact0_s.read_length       = 0x1;
	xact0_s.xact_active       = 0x1;
	xact0_s.xact_success      = 0x0;

	start_i2c_master_xact(c_i2c_channel, &xact0_s, &c_xact_callback);
	c_poll_wait(c_i2c_channel);
	return (xact0_s.i2c_rd_buffer[0] & C_RDY) == C_RDY;
}


void LSM303DLH_m_get_data(XACT_FnCallback* xact_callback){
	i2c_master_xact_t       gyrodata;
	gyrodata.i2c_tx_buffer[0]  = i2c_create_write_address(LSM303DLH_ADDR_M);
	gyrodata.i2c_tx_buffer[1]  = C_OUT_X_L_M | C_AUTO_INCREMENT; //Auto increment through, temperature, data status,
	gyrodata.write_length      = 0x2;					   //then all the axis data registers
	gyrodata.i2c_tx_buffer[2] =  i2c_create_read_address(LSM303DLH_ADDR_M);
	gyrodata.read_length       = 0x7;
	gyrodata.xact_active       = 0x1;
	gyrodata.xact_success      = 0x0;

	start_i2c_master_xact(c_i2c_channel, &gyrodata, c_xact_callback);
}

void LSM303DLH_init_m(i2c_iface i2c_ch){
	i2c_master_xact_t       gyroinit;

	c_i2c_channel = i2c_ch;

	gyroinit.i2c_tx_buffer[0]  = i2c_create_write_address(LSM303DLH_ADDR_M);
    gyroinit.i2c_tx_buffer[1]  = C_CRA_REG_M | C_AUTO_INCREMENT;
    gyroinit.i2c_tx_buffer[2]  = C_ODR_3 & C_CRA_REG_M_MASK; 	//CRA_REG_M
    gyroinit.i2c_tx_buffer[3]  = C_RANGE_4 & C_CRB_REG_M_MASK; 	//CRB_REG_M
    gyroinit.i2c_tx_buffer[4]  = C_CONT_CONV & C_MR_REG_M_MASK;	//MR_REG_M
    gyroinit.write_length      = 0x5;
    gyroinit.read_length       = 0x0;
    gyroinit.xact_active       = 0x1;

    start_i2c_master_xact(c_i2c_channel, &gyroinit, &c_nullcallback);
}

