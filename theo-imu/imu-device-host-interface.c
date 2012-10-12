/*
 * imu-device-host-interface.c
 *
 *  Created on: Jul 19, 2012
 *      Author: theo
 */


#include "imu-device-host-interface.h"


void fill_imu_packet(imuPacket * pkt, unsigned char * buf){
	pkt->ID = buf[0];
	pkt->timestamp = ((uint32_t)buf[1]<<24) |
			        ((uint32_t)buf[2]<<16) |
			        ((uint32_t)buf[3]<<8)  |
			        ((uint32_t)buf[4]);
	pkt->status = buf[5];
	pkt->x = (int16_t)((uint16_t)buf[6]) | ((uint16_t)buf[7]<<8);
	pkt->y = (int16_t)((uint16_t)buf[8]) | ((uint16_t)buf[9]<<8);
	pkt->z = (int16_t)((uint16_t)buf[10])| ((uint16_t)buf[11]<<8);
	pkt->extra_data = buf[12];
}

//todo: combine fill and submit?

/*
 * Device side
 */

void submit_imu_packet(imuPacket * pkt, int (*p_putc)(char)){
	p_putc(pkt->ID);

    p_putc( pkt->timestamp &  0xff);
    p_putc((pkt->timestamp & (0xff << 8)) >> 8);
    p_putc((pkt->timestamp & (0xff << 16)) >> 16);
    p_putc((pkt->timestamp & (0xff << 24)) >> 24);

    p_putc(pkt->status);

    p_putc( pkt->x &  0xff);
    p_putc((pkt->x & (0xff << 8)) >> 8);
    p_putc( pkt->y &  0xff);
    p_putc((pkt->y & (0xff << 8)) >> 8);
    p_putc( pkt->z &  0xff);
    p_putc((pkt->z & (0xff << 8)) >> 8);

    p_putc(pkt->extra_data);
}

void copy_imu_packet(imuPacket * destination, imuPacket * source){
	destination->ID         = source->ID;
	destination->timestamp  = source->timestamp;
	destination->status     = source->status;
	destination->x          = source->x;
	destination->y          = source->y;
	destination->z          = source->z;
	destination->extra_data = source->extra_data;
}

const char * imu_pkt_id_to_str(imuPacket * pkt){
	switch(pkt->ID){
	case ADDR_ACC:
		return "ACC";
	case ADDR_GYR:
		return "GYR";
	case ADDR_MAG:
		return "MAG";
	default:
		return "unknown ID";
	}
}
/*
 * Host side
 */
void print_packet(imuPacket * pkt){ //second arg fprint like func?

}
