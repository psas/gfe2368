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

bool getting_data = false;

void adis_cb(spi_master_xact_data* caller, spi_master_xact_data* spi_xact, void* data){
    //REQUIRES ADDITIONAL PYLONS


//    int bytes_sent = USBHwEPWrite (BULK0_IN_EP, spi_xact->readbuf, ADIS_PACKET_LENGTH);
//    if(bytes_sent != ADIS_PACKET_LENGTH){
//        uart0_putstring("\n***ADIS WRITE DATA TO USB FAILED***\n");
//    }
    getting_data = false;
}


//static bool adis_ctrl(TSetupPacket *pSetup, int *piLen, uint8_t **ppbData){
//    //! \todo: set sensors to idle/power_down on INST_STOP
//    switch(IMU_INST(pSetup->bRequest)){
//    case INST_RESET:
//        break;
//    case INST_GO:
//        IO2IntEnR |= ADIS_DRDY;
//        GREEN_LED_ON;
//        break;
//    case INST_STOP:
//        IO0IntEnR &= ~(ADIS_DRDY);
//        GREEN_LED_OFF;
//        break;
//    case INST_SET_SPEED:
//        break;
//    default:
//        break;
//    }
//    return true;
//}

void adis_isr(void) {
    //uint32_t timestamp = T0TC;
    uart0_putstring("\n***adis_isr***\n");

    //IO0IntStatR for pin interrupt status
    //IO0IntClr to clear interrupt
    if((IO2IntStatR & ADIS_DRDY) ){
        //uart0_putstring("L3G\n");
        //ADIS_timestamp = timestamp;
        adis_read_brst_mode(adis_cb);
        IO2IntClr = ADIS_DRDY;
    }
    EXIT_INTERRUPT;
}

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

//    USBInit(imu_descriptor);
//    uint8_t abClassReqData[MAX_PACKET_SIZE0];
//    USBRegisterRequestHandler(REQTYPE_TYPE_VENDOR, adis_ctrl, abClassReqData);
//    USBHwConnect(true);

	adis_reset();

	//  printf_lpc(UART0, "\n*** Init pins for ADIS\r\n");
	adis_init();
//	printf_lpc(UART0, "PINSEL4 is: 0x%x\n", PINSEL4);
//	printf_lpc(UART0, "PINMODE4 is: 0x%x\n", PINMODE4);
	/*! user manual p171: GPIO0 and GPIO2 interrupts share the same VIC slot with the
	 *   External Interrupt 3 event.
	 */

	//  printf_lpc(UART0, "\n*** Reset ADIS\r\n");

	//  printf_lpc(UART0,"\n*** Initialize SPI ***\r\n" );

	//  dummy_spi_xact();

	//adis_read_smpl_prd();

	//adis_read_id();
	adis_read_gpio_ctl();

	adis_read_id();


	adis_read_brst_mode(adis_cb);
	adis_read_brst_mode(adis_cb);


//	adis_read_brst_mode();

	//util_wait_msecs(1000);  // printf gets in the way of SPI interrupt or vice-versa.

//
//		VIC_SET_EINT3_GPIO_HANDLER(adis_isr);
//		ENABLE_INT(VIC_EINT3_GPIO);
//		IO2IntEnR = (1<<10);

		while(1) {
	    color_led_flash(5, RED_LED, FLASH_FAST);
//		adis_process_done_q();
//		// printf_lpc(UART0,"2 SLOW flashes...red, blue then green\r\n");
		printf_lpc(UART0,".");
//		//  adis_process_done_q();
//		//       color_led_flash(2, RED_LED, FLASH_NORMAL ) ;
//		//        RED_LED_OFF;
//		//        color_led_flash(2, BLUE_LED,  FLASH_NORMAL ) ;
//		//        BLUE_LED_OFF;
//		color_led_flash(2, GREEN_LED, FLASH_NORMAL ) ;

	}

	return(0);
}

