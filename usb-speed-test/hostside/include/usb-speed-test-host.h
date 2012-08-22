

#ifndef USB_SPEED_TEST_HOST_H_
#define USB_SPEED_TEST_HOST_H_

#define NUM_IFACES	1

#define NUM_EPS					32

#define CTRL_IN_EP				0x80
#define CTRL_OUT_EP				0x00
#define INTR_IN_EP              0x81
#define INTR_OUT_EP				0x01
#define BULK_IN_EP              0x82
#define BULK_OUT_EP             0x02
#define ISOC_IN_EP				0x83
#define ISOC_OUT_EP				0x03

//see lpc23xx user manual p318
/** convert from endpoint address to endpoint index */
#define EP2IDX(bEP) ((((bEP)&0xF)<<1)|(((bEP)&0x80)>>7))
/** convert from endpoint index to endpoint address */
#define IDX2EP(idx) ((((idx)<<7)&0x80)|(((idx)>>1)&0xF)

#define CTRL_IN_IDX				EP2IDX(CTRL_IN_EP)
#define CTRL_OUT_IDX			EP2IDX(CTRL_OUT_EP)
#define INTR_IN_IDX             EP2IDX(INTR_IN_EP)
#define INTR_OUT_IDX			EP2IDX(INTR_OUT_EP)
#define BULK_IN_IDX             EP2IDX(BULK_IN_EP)
#define BULK_OUT_IDX            EP2IDX(BULK_OUT_EP)
#define ISOC_IN_IDX				EP2IDX(ISOC_IN_EP)
#define ISOC_OUT_IDX			EP2IDX(ISOC_OUT_EP)


#define MAX_PACKET_SIZE 		64

#define STOPPED  0x00
#define CTRL_REQ 0x01
#define INTR_REQ 0x02
#define BULK_REQ 0x04
#define ISOC_REQ 0x08
#define READ_DATA 0x10

#endif /* USB_SPEED_TEST_HOST_H_ */
