/* \file at45161D.h
 */

#ifndef _AT45161D_H
#define _AT45161D_H

#include "lpc23xx-spi.h"
#include "lpc23xx-util.h"

#include "ringbuffer.h"

extern    Ringbuffer       at45_tx_done_q;

#define     AT45_RESET_HIGH           (FIO1SET |= (1<<9))
#define     AT45_RESET_LOW            (FIO1CLR |= (1<<9))


#define     P1_9_PINSEL2              18
#define     GPIO_PINSEL2              0b00

//P1.9
#define     PINDIR_AT45_RESET_OUTPUT     (FIO1DIR  = (FIO1DIR | (1<<9)) )
#define     PINSEL_AT45_RESET            (PINSEL2  = (PINSEL2  & ~(0x3 << P1_9_PINSEL2)) | (GPIO_PINSEL2    << P1_9_PINSEL2))
#define     PINMODE_AT45_RESET_PULLUP    (PINMODE2 = (PINMODE2 & ~(0x3 << P1_9_PINSEL2)) | (P0_PULLUP_MODE  << P1_9_PINSEL2))

typedef enum {
    AT45_ID_OPCODE    = 0x09f
} at45_opcode;

typedef struct {
    at45_opcode cmd;
    uint8_t     readbytes;
} at45_spi_xact;

extern   spi_master_xact_data      at45_id_xact;

void at45_init() ;
void at45_reset();

void at45_read_id() ;

void at45_read_intr(at45_spi_xact* s) ;

#endif
