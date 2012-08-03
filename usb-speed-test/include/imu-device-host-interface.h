/*
 * imu-device-host-interface.h
 *
 *  Created on: Jul 19, 2012
 *      Author: theo
 */

#ifndef IMU_DEVICE_HOST_INTERFACE_H_
#define IMU_DEVICE_HOST_INTERFACE_H_

#define NUM_EPS					32

#define CTRL_IN_EP				0x80
#define CTRL_OUT_EP				0x00
#define INTR_IN_EP              0x81
#define INTR_OUT_EP				0x01
#define BULK_IN_EP              0x82
#define BULK_OUT_EP             0x02
#define ISOC_IN_EP				0x83
#define ISOC_OUT_EP				0x03

#define MAX_PACKET_SIZE 		64

#endif /* IMU_DEVICE_HOST_INTERFACE_H_ */
