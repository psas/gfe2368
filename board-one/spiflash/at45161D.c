/*! \file at45161D.c
 * interface file for Atmel 16 mbit spi flash
 */

#include "lpc23xx-util.h"
#include "ringbuffer.h"
#include "printf-lpc.h"
#include "gfe2368-util.h"

#include "at45161D.h"

Ringbuffer                at45_tx_done_q;

spi_master_xact_data      at45_id_xact;

at4516D_cache             at45_data_cache;


bool at45_read_cache(at45_opcode at45_recent, uint16_t* at45_data) {

	at4516D_cache*   c;
	c               = &at45_data_cache;

	if((at45_recent==AT45_ID_OPCODE)) {
		at45_data[0] = c->at45_man_id.data;
		at45_data[1] = c->at45_family_code.data;
		at45_data[2] = c->at45_mlc_code.data;
		return true;
	}

	return false;
}

/*! \brief Initialize IO for at45
 *
 */
void at45_init() {

	rb_initialize(&at45_tx_done_q);

	FIO_ENABLE;
	PINDIR_AT45_RESET_OUTPUT;
	PINSEL_AT45_RESET;
	PINMODE_AT45_RESET_PULLUP;
    AT45_RESET_HIGH;
}


void at45_get_data() {

	bool            ok            = false;

	at45_opcode     at45_recent  = 0;

	uint16_t        at45_data[AT45_MAX_DATA_BUFFER];

	ok = rb_get_elem((uint8_t *) &at45_recent, &at45_tx_done_q);
	if(ok) {
		ok = at45_read_cache(at45_recent, at45_data);
		if(ok) {
			switch(at45_recent) {

			case AT45_ID_OPCODE:
				printf_lpc(UART0, "\r\n at45 ID is: 0x%x\r\n",      at45_data[0]) ;
				printf_lpc(UART0, "\r\n at45 family  is: 0x%x\r\n", at45_data[1]) ;
				break;
			default:
				break;

			}
		}
	}
}

void at45_process_done_q() {
	while(!rb_is_empty(&at45_tx_done_q)) {
		at45_get_data();
	}
}
/*! \brief Reset the at45
 *
 */
void at45_reset() {
	AT45_RESET_LOW;

	util_wait_msecs(200);

	AT45_RESET_HIGH;
}


/*! \brief Interrupt Callback function for read
*/
void at45_read_id_cb(spi_master_xact_data* caller, spi_master_xact_data* spi_xact, void* data) {
    // data is NULL in this cb.
	// if (data != NULL) {}

	uint16_t i           = 0;
    at45_opcode  opcode  = 0;

    // copy out read buffer.
    for(i=0; i<spi_xact->read_numbytes; ++i) {
        caller->readbuf[i] = spi_xact->readbuf[i];
    }
    // copy out write buffer.
    for(i=0; i<spi_xact->write_numbytes; ++i) {
        caller->writebuf[i] = spi_xact->writebuf[i];
    }

    // The register address is always the first byte of the write buffer.
    opcode = caller->writebuf[0];

    switch(opcode) {
        case AT45_ID_OPCODE:
        	BLUE_LED_ON;
        	at45_data_cache.at45_family_code.data      = caller->readbuf[0];
        	at45_data_cache.at45_man_id.data           = caller->readbuf[1];
        	at45_data_cache.at45_mlc_code.data         = caller->readbuf[2];
        	break;
        default:
            break;
    }
    if(!rb_is_full(&at45_tx_done_q)) {
        rb_put_elem((char) opcode, &at45_tx_done_q);
    }       // now check queue not empty in main loop to see if there is fresh data.

}

void at45_read_id() {

	at45_spi_xact    at45_id_data;

    at45_id_data.cmd        = AT45_ID_OPCODE;
    at45_id_data.readbytes  = 3;

    at45_read_intr(&at45_id_data);

}


/*! \brief Interrupt driven version of read at45.
 *
 * For SPI interrupt in the at45, SPI is set up for MSB first.
 *
 * After reset, the at45 is set for SPI MODE 3
 *
 * Configure structure at45_xact prior to calling function.
 */
void at45_read_intr(at45_spi_xact* s) {

    spi_init_master_xact_data(&at45_id_xact);

    at45_id_xact.writebuf[0]     = s->cmd;
    at45_id_xact.write_numbytes  = 1;
    at45_id_xact.read_numbytes   = s->readbytes;
    at45_id_xact.dummy_value     = 0xaa;

    start_spi_master_xact_intr(&at45_id_xact, &at45_read_id_cb) ;
}



