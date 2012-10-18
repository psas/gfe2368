/*
 * L3G4200D.c
 *
 */

#include <stdlib.h>
#include <stdint.h>

#include "lpc23xx.h"
#include "lpc23xx-i2c.h"

#include "L3G4200D.h"

static i2c_iface i2c_channel;

void L3G4200D_init(i2c_iface i2c_ch){
	i2c_master_xact_t init;

	i2c_channel = i2c_ch;

	init.device_addr = L3G4200D_ADDR;
    init.tx_buffer[0] = G_CTRL_REG1 | G_AUTO_INCREMENT;
    init.tx_buffer[1] = G_ODR_100 | G_PWR | G_ALL_AXIS_ENABLE; //CTRL_REG1, turn on, data rate 100Hz
    init.tx_buffer[2] = G_CTRL_REG2_DEFAULT;
    init.tx_buffer[3] = G_INT2_DRDY;     //CTRL_REG3, interrupt 2 fires on data ready
//    init.i2c_tx_buffer[4]  = G_BDU; //CTRL_REG4 block data update
    init.write_length = 4;
    init.read_length = 0;

    start_i2c_master_xact(i2c_channel, &init, NULL);
}


void L3G4200D_get_data(i2c_callback* cb){
	i2c_master_xact_t data;

	data.device_addr = L3G4200D_ADDR;
	//Auto increment through temperature, data status, then all the axis data registers
	data.tx_buffer[0] = G_OUT_TEMP | G_AUTO_INCREMENT;
	data.write_length = 1;
	data.read_length = 8;

	start_i2c_master_xact(i2c_channel, &data, cb);
}

int L3G4200D_data_overrun(uint8_t status_reg){
	return ((status_reg & G_ZYXOR) == G_ZYXOR);
}

int L3G4200D_data_available(uint8_t status_reg){
	return ((status_reg & G_ZYXDA) == G_ZYXDA);
}

int L3G4200D_set_ctrl_reg(int reg, uint8_t val){
	i2c_master_xact_t gyro;
	uint8_t reg_addr;
	switch(reg){
	case 1:
		reg_addr = G_CTRL_REG1;
		break;
	case 2:
		reg_addr = G_CTRL_REG2;
		break;
	case 3:
		reg_addr = G_CTRL_REG3;
		break;
	case 4:
		reg_addr = G_CTRL_REG4;
		break;
	case 5:
		reg_addr = G_CTRL_REG5;
		break;
	default:
		return -1;
	}


	gyro.device_addr = L3G4200D_ADDR;
	gyro.tx_buffer[0] = reg_addr;
	gyro.tx_buffer[1] = val;
	gyro.write_length = 2;

	start_i2c_master_xact(i2c_channel, &gyro, NULL);
	return 0;
}
