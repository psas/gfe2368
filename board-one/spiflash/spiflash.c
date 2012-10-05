/*! \file spiflash.c
*/

#include <stdio.h>
#include <stdint.h>

#include "lpc23xx.h"

#include "lpc23xx-pll.h"
#include "lpc23xx-uart.h"
#include "lpc23xx-util.h"
#include "lpc23xx-spi.h"
#include "printf-lpc.h"

#include "gfe2368-info.h"
#include "gfe2368-util.h"

#include "at45161D.h"
#include "spiflash.h"

int main (void) {

    pllstart_seventytwomhz() ;
    //  pllstart_sixtymhz() ;
   // pllstart_fourtyeightmhz() ;

    info_init();

    uart0_init_115200() ;

    printf_lpc(UART0,"\n***Starting spiflash test ***\r\n\r\n");

    printf_lpc(UART0,"\n***Board is defined as: %s ***\r\n", infoquery_gfe_boardtag() );

    init_color_led();

    RED_LED_OFF;
    BLUE_LED_OFF;
    GREEN_LED_OFF;

    printf_lpc(UART0,"\n*** Initialize SPI ***\r\n" );

    at45_init();
    at45_reset();

    spi_init_master_intr(CCLK_DIV1, SPI_100KHZ);

    at45_read_id();

    while(1) {
        printf_lpc(UART0,"2 SLOW flashes...red, blue then green\r\n");
      //  color_led_flash(2, RED_LED, FLASH_SLOW ) ;
      //  RED_LED_OFF;
      //  color_led_flash(2, BLUE_LED,  FLASH_SLOW ) ;
      //  BLUE_LED_OFF;
        color_led_flash(2, GREEN_LED, FLASH_SLOW ) ;
        GREEN_LED_OFF;
    }

    return(0);
}

