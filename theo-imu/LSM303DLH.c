/*
 * LSM303DLH.c
 *
 */


#include <stdlib.h>
#include <stdint.h>

#include "lpc23xx.h"
#include "lpc23xx-i2c.h"

#include "LSM303DLH.h"


static i2c_iface i2c_channel;

void LSM303DLH_m_get_data(i2c_callback * callback){
	i2c_master_xact_t data;
	data.device_addr = LSM303DLH_ADDR_M;
	//Auto increment through all the data registers, then status register.
	data.tx_buffer[0] = C_OUT_X_L_M | C_AUTO_INCREMENT;
	data.write_length = 1;
	data.read_length = 7;

	start_i2c_master_xact(i2c_channel, &data, callback);
}

void LSM303DLH_init_m(i2c_iface i2c_ch){ //todo:error cb
	i2c_master_xact_t init;
	i2c_channel = i2c_ch;
	init.device_addr = LSM303DLH_ADDR_M;
    init.tx_buffer[0] = C_CRA_REG_M | C_AUTO_INCREMENT;
    init.tx_buffer[1] = C_ODR_0_75 & C_CRA_REG_M_MASK; 	//CRA_REG_M
    init.tx_buffer[2] = C_RANGE_1_9 & C_CRB_REG_M_MASK; //CRB_REG_M
    init.tx_buffer[3] = C_CONT_CONV & C_MR_REG_M_MASK;	//MR_REG_M
    init.write_length = 4;
    init.read_length  = 0;

    start_i2c_master_xact(i2c_channel, &init, NULL);
}

int LSM303DLH_m_set_ctrl_reg(int reg, uint8_t val){
	i2c_master_xact_t mag;
	uint8_t reg_addr;
	switch(reg){
	case 1:
		reg_addr = C_CRA_REG_M;
		break;
	case 2:
		reg_addr = C_CRB_REG_M;
		break;
	case 3:
		reg_addr = C_MR_REG_M;
		break;
	default:
		return 0;
	}

	mag.device_addr= LSM303DLH_ADDR_M;
	mag.tx_buffer[0] = reg_addr;
	mag.tx_buffer[1] = val;
	mag.write_length = 2;

	start_i2c_master_xact(i2c_channel, &mag, NULL);
	return 1;
}

