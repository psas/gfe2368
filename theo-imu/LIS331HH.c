/*
 * LIS331HH.c
 * LPC23xx API LIS331HH accelerometer
 */

#include <stdlib.h>
#include <stdint.h>

#include "lpc23xx.h"
#include "lpc23xx-i2c.h"

#include "LIS331HH.h"

static i2c_iface i2c_channel = I2C1;

//TODO: self-test

void LIS331HH_init(i2c_iface i2c_ch){	//todo: error cb
	i2c_master_xact_t init;
	i2c_channel = i2c_ch;

    //ctrl register setup
	init.device_addr = LIS331HH_ADDR;
	init.tx_buffer[0] = A_CTRL_REG1 | A_AUTO_INCREMENT; //initial ctrl register addr
	init.tx_buffer[1] = A_LPWR_ODR_05 | A_ALL_AXIS_ENABLE; //CTRL_REG1 data
	init.tx_buffer[2] = A_CTRL_REG2_DEFAULT;
	init.tx_buffer[3] = A_INT_ACTIVE_HIGH | A_INT1_DRDY | A_INT1_LATCH; //interupt 1 fires on data ready
	init.tx_buffer[4] = A_BDU;
	init.write_length = 5;
	init.read_length  = 0;
    start_i2c_master_xact(i2c_channel, &init, NULL);
}

void LIS331HH_get_data(i2c_callback * cb){
	i2c_master_xact_t data;

	data.device_addr = LIS331HH_ADDR;
	data.tx_buffer[0] = A_STATUS_REG | A_AUTO_INCREMENT;
	data.write_length = 1;
	data.read_length = 7;

	start_i2c_master_xact(i2c_channel, &data, cb);
}

int LIS331HH_data_overrun(uint8_t status_reg){
	return ((status_reg & A_ZYXOR) == A_ZYXOR);
}

int LIS331HH_data_available(uint8_t status_reg){
	return ((status_reg & A_ZYXDA) == A_ZYXDA);
}

int LIS331HH_set_ctrl_reg(int reg, uint8_t val){
	i2c_master_xact_t accel;
	uint8_t reg_addr;
	switch(reg){
	case 1:
		reg_addr = A_CTRL_REG1;
		break;
	case 2:
		reg_addr = A_CTRL_REG2;
		break;
	case 3:
		reg_addr = A_CTRL_REG3;
		break;
	case 4:
		reg_addr = A_CTRL_REG4;
		break;
	case 5:
		reg_addr = A_CTRL_REG5;
		break;
	default:
		return 0;
	}

	accel.device_addr = LIS331HH_ADDR;
	accel.tx_buffer[0] = reg_addr;
	accel.tx_buffer[1] = val;
	accel.write_length = 2;

	start_i2c_master_xact(i2c_channel, &accel, NULL);
	return 1;
}
