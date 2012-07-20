/*
 * imu-device-host-interface.c
 *
 *  Created on: Jul 19, 2012
 *      Author: theo
 */


#include "imu-device-host-interface.h"


void fill_imu_packet(imuPacket * pkt, uint8_t ID, uint32_t timestamp, uint8_t status,
		int16_t x, int16_t y, int16_t z, uint8_t extra_data){
	pkt->ID = ID;
	pkt->timestamp = timestamp;
	pkt->status = status;
	pkt->x = x;
	pkt->y = y;
	pkt->z = z;
	pkt->extra_data = extra_data;
}


//todo: combine the two?

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

/*
 * Host side
 */
void print_packet(imuPacket * pkt){ //second arg fprint like func?

}
