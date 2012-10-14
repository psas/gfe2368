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

#define ADDR_ACC                0x80
#define ADDR_GYR                0x40
#define ADDR_MAG                0x20
#define ADDR_CAC                0x10
#define ADDR_ALL                0xF0
#define INST_RESET              0x01
#define INST_GO                 0x02
#define INST_STOP               0x03
#define INST_INC_SPEED          0x04
#define INST_DEC_SPEED          0x05

#define IMU_INST(X)             ((X) & 0x0F)
#define IMU_ADDR(X)             ((X) & 0xF0)


#define IMU_PACKET_LENGTH 13 //todo: sizeof(imuPacket)?
#define MAX_PACKET_SIZE 		64

typedef struct imuPacket {
	uint8_t ID;
	uint32_t timestamp;
	uint8_t status;
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t extra_data;
} imuPacket;


void fill_imu_packet(imuPacket * pkt, unsigned char * buf);
void submit_imu_packet(imuPacket * pkt, int (*p_putc)(char));
void copy_imu_packet(imuPacket * destination, imuPacket * source);
const char * imu_pkt_id_to_str(imuPacket * pkt);



#endif /* IMU_DEVICE_HOST_INTERFACE_H_ */
