/*
 * L3G4200D.h
 *
 *  Created on: Nov 19, 2011
 *      Author: theo
 */

#ifndef IMU_H_
#define IMU_H_

#include "lpc23xx-types.h"

void IMU_isr() __attribute__ ((interrupt("IRQ")));

#define VIC_GPIO_BIT        (17)
#define ENABLE_GPIO_INT     (VICIntEnable |= (1<<VIC_GPIO_BIT))
#define DISABLE_GPIO_INT    (VICIntEnClr   = (1<<VIC_GPIO_BIT))
#define RAISE_GPIO_INT      (VICSoftInt   |= (1<<VIC_GPIO_BIT))
#define CLR_SW_GPIO_INT     (VICSoftIntClr = (1<<VIC_GPIO_BIT))

typedef struct axis_data{
	int16_t x;
	int16_t y;
	int16_t z;
	BOOL modified;
} axis_data;
/*GPIO 0 wiring*/
//P0.00 CAN
//P0.01 CAN
//P0.02 UART
//P0.03 UART
//P0.04 CAN
//P0.05 ----
//P0.06 ---- = ACCEL SA0
//P0.07 ---- = ACCEL INT2
//P0.08 ---- = ACCEL INT1
//P0.09 ---- = ACCEL CS
//P0.10 SDA2 = MAG SDA
//P0.11 SCL2 = MAG SCL
//-----
//P0.15 ---- = MAG INT1
//P0.16 ----
//P0.17 ----
//P0.18 ---- = MAG INT2
//P0.19 SDA1 = ACCEL SDA
//P0.20 SCL1 = ACCEL SCL
//P0.21 ---- = MAG SA
//P0.22 ---- = MAG DRDY
//P0.23 ---- = GYRO INT1
//P0.24 ---- = GYRO INT2
//P0.25 ---- = GYRO CS
//P0.26 ---- = GYRO SA0
//P0.27 SDA0 = GYRO SDA
//P0.28 SCL0 = GYRO SCL
//P0.29 USB
//P0.30 USB

#define ACCEL_SA0	(1<<6)
#define ACCEL_INT2	(1<<7)
#define ACCEL_INT1	(1<<8)
#define ACCEL_CS	(1<<9)
#define MAG_INT1	(1<<15)
#define MAG_INT2	(1<<18)
#define MAG_SA		(1<<21)
#define MAG_DRDY	(1<<22)
#define GYRO_INT1	(1<<23)
#define GYRO_INT2	(1<<24)
#define GYRO_CS		(1<<25)
#define	GYRO_SA0	(1<<26)

#define L3G4200D_STUCK		((IO0IntEnR & GYRO_INT2) && (FIO0PIN & GYRO_INT2) && !is_binsem_locked(&i2c0_binsem_g))
#define LIS331HH_STUCK		((IO0IntEnR & ACCEL_INT1)&& (FIO0PIN & ACCEL_INT1)&& !is_binsem_locked(&i2c1_binsem_g))
#define LSM303DLH_M_STUCK	((IO0IntEnR & MAG_DRDY)  && (FIO0PIN & MAG_DRDY)  && !is_binsem_locked(&i2c2_binsem_g))

#endif /* IMU_H_ */
