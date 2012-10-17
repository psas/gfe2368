/*! \file adis-dev.c
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

#include "adis.h"
#include "adis-dev.h"

int main (void) {

	pllstart_seventytwomhz() ;
	//  pllstart_sixtymhz() ;
	//pllstart_fourtyeightmhz() ;

	info_init();

	uart0_init_115200() ;

	printf_lpc(UART0,"\n***Starting adis-dev test ***\r\n\r\n");

	printf_lpc(UART0,"\n***Board is defined as: %s ***\r\n", infoquery_gfe_boardtag() );

	init_color_led();

	RED_LED_OFF;
	BLUE_LED_OFF;
	GREEN_LED_OFF;

	// util_wait_msecs(2000);
	adis_spi_ctl.spi_cpol_val = SPI_SCK_ACTIVE_HIGH;
	adis_spi_ctl.spi_cpha_val = SPI_SCK_SECOND_CLK;
	adis_spi_ctl.spi_lsbf_val = SPI_DATA_MSB_FIRST;

	spi_init_master_intr(CCLK_DIV1, SPI_500KHZ, &adis_spi_ctl);

	SCK_HIGH;

	//  printf_lpc(UART0, "\n*** Init pins for ADIS\r\n");
	adis_init();

	//  printf_lpc(UART0, "\n*** Reset ADIS\r\n");
	adis_reset();

	//  printf_lpc(UART0,"\n*** Initialize SPI ***\r\n" );

	//  dummy_spi_xact();

	adis_read_smpl_prd();

	adis_read_id();

	adis_read_brst_mode();

	util_wait_msecs(1000);  // printf gets in the way of SPI interrupt or vice-versa.

	while(1) {
		adis_process_done_q();
		// printf_lpc(UART0,"2 SLOW flashes...red, blue then green\r\n");
		printf_lpc(UART0,".");
		//  adis_process_done_q();
		//       color_led_flash(2, RED_LED, FLASH_NORMAL ) ;
		//        RED_LED_OFF;
		//        color_led_flash(2, BLUE_LED,  FLASH_NORMAL ) ;
		//        BLUE_LED_OFF;
		color_led_flash(2, GREEN_LED, FLASH_NORMAL ) ;
	}

	return(0);
}

