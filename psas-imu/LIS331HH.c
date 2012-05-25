
/*
 * LIS331HH.c
 * testing program for LIS331HH accelerometer
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
#include "LIS331HH.h"

i2c_iface 				a_i2c_channel;

/*
 * xact_callback
 * callback function for i2c
 * Generic
 */
void a_xact_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s) {
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

void a_empty_callback(i2c_master_xact_t* caller, i2c_master_xact_t* i2c_s) {
	if(!(i2c_s->xact_success)){
		uart0_putstring("\n***LIS331HH I2C call failed***\n");
	}
	return;
}


void a_poll_wait(i2c_iface i2c_ch) {
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

/*checks LIS331HH to see if a new set of data is available.
 * see section 7.8, p29 of the LIS331HH data sheet
 * */
int accel_data_rdy(){
	i2c_master_xact_t       xact0_s;

	xact0_s.i2c_tx_buffer[0]  = i2c_create_write_address(LIS331HH_ADDR);
	xact0_s.i2c_tx_buffer[1]  = A_STATUS_REG; //data status register register addr
	xact0_s.write_length      = 0x2;
	xact0_s.i2c_tx_buffer[2] =  i2c_create_read_address(LIS331HH_ADDR);
	xact0_s.read_length       = 0x1;
	xact0_s.xact_active       = 0x1;
	xact0_s.xact_success      = 0x0;

	start_i2c_master_xact(a_i2c_channel, &xact0_s, &a_xact_callback);
	a_poll_wait(a_i2c_channel);
	return (xact0_s.i2c_rd_buffer[0] & A_ZYXDA) == A_ZYXDA;
}



void LIS331HH_get_data(XACT_FnCallback* xact_fn){
	i2c_master_xact_t       xact0_s;

	xact0_s.i2c_tx_buffer[0]  = i2c_create_write_address(LIS331HH_ADDR);
	xact0_s.i2c_tx_buffer[1]  = A_OUT_X_L | A_AUTO_INCREMENT;
	xact0_s.write_length      = 0x2;
	xact0_s.i2c_tx_buffer[2] =  i2c_create_read_address(LIS331HH_ADDR);
	xact0_s.read_length       = 0x6;
	xact0_s.xact_active       = 0x1;
	xact0_s.xact_success      = 0x0;

	start_i2c_master_xact(a_i2c_channel, &xact0_s, xact_fn);
}


void LIS331HH_init(i2c_iface i2c_ch){//return isr?
	a_i2c_channel=i2c_ch;
	i2c_master_xact_t accel_init;
	//mode setting poll vs interupt?

    //ctrl register setup
	accel_init.i2c_tx_buffer[0]  = i2c_create_write_address(LIS331HH_ADDR);
	accel_init.i2c_tx_buffer[1]  = A_CTRL_REG1 | A_AUTO_INCREMENT; //initial ctrl register addr
	accel_init.i2c_tx_buffer[2]  = A_LPWR_ODR_1 | A_ALL_AXIS_ENABLE; //CTRL_REG1 data
	accel_init.i2c_tx_buffer[3]  = A_CTRL_REG2_DEFAULT;
	accel_init.i2c_tx_buffer[4]  = A_INT_ACTIVE_LOW | A_INT1_DRDY | A_INT1_LATCH; //interupt 1 fires on data ready and is latched
	accel_init.write_length      = 0x5;
	accel_init.read_length       = 0x0;
	accel_init.xact_active       = 0x1;
	accel_init.xact_success      = 0x0;
    //todo:enable block data update
    start_i2c_master_xact(a_i2c_channel, &accel_init, &a_empty_callback);
}
