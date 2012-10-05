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


/*! \brief Initialize IO for at45
 *
 */
void at45_init() {
	FIO_ENABLE;
	PINDIR_AT45_RESET_OUTPUT;
	PINSEL_AT45_RESET;
	PINMODE_AT45_RESET_PULLUP;
    AT45_RESET_HIGH;
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
void at45_read_id_cb(spi_master_xact_data* caller, spi_master_xact_data* spi_xact) {
    uint16_t i           = 0;
    uint8_t  opcode      = 0;

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

    printf_lpc(UART0, "%s: Called\r\n", __func__);

    at45_id_xact.writebuf[0]     = s->cmd;
    at45_id_xact.write_numbytes  = 1;
    at45_id_xact.read_numbytes   = s->readbytes;
    at45_id_xact.dummy_value     = 0xaa;

    start_spi_master_xact_intr(&at45_id_xact, &at45_read_id_cb) ;
}



