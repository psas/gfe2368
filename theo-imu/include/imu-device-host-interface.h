/*
 * imu-device-host-interface.h
 *
 *  Created on: Jul 19, 2012
 *      Author: theo
 */

#ifndef IMU_DEVICE_HOST_INTERFACE_H_
#define IMU_DEVICE_HOST_INTERFACE_H_

#include <stdint.h>

#define BULK_OUT_EP             0x05
#define BULK_IN_EP              0x82
#define ACCEL	1
#define GYRO	2
#define MAG		3
#define IMU_PACKET_LENGTH 13 //todo: sizeof(imuPacket)?

typedef struct imuPacket {
	uint8_t ID;
	uint32_t timestamp;
	uint8_t status;
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t extra_data;
} imuPacket;

void fill_imu_packet(imuPacket * pkt, uint8_t ID, uint32_t timestamp, uint8_t status,
		int16_t x, int16_t y, int16_t z, uint8_t extra_data);
void submit_imu_packet(imuPacket * pkt, int (*p_putc)(char));


#endif /* IMU_DEVICE_HOST_INTERFACE_H_ */
