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

#include "ADIS16405.h"
#include "adis-usb-bulk.h"

bool getting_data = false;

void adis_dev_cb(spi_master_xact_data* caller, spi_master_xact_data* spi_xact, void* data){
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

/*! \brief enable the DIO1 interrupt from ADIS
 *
 */
static void adis_dev_enable_dio1() {
	/*! user manual p171: GPIO0 and GPIO2 interrupts share the same VIC slot with the
	 *   External Interrupt 3 event.
	 */
	VIC_SET_EINT3_GPIO_HANDLER(adis_dev_isr);
	ENABLE_INT(VIC_EINT3_GPIO);
	IO2IntEnR = (1<<10);
}

void adis_dev_isr(void) {
	//uint32_t timestamp = T0TC;
	//uart0_putstring("*");
	//IO0IntStatR for pin interrupt status
	//IO0IntClr to clear interrupt
	if((IO2IntStatR & ADIS_DRDY) ){
		//uart0_putstring("L3G\n");
		//ADIS_timestamp = timestamp;
		adis_read_brst_mode(adis_dev_cb);
		IO2IntClr = ADIS_DRDY;
	}
	EXIT_INTERRUPT;
}

int main (void) {

	pllstart_seventytwomhz() ;
	//  pllstart_sixtymhz() ;
	//  pllstart_fourtyeightmhz() ;

	info_init();

	uart0_init_115200() ;

	printf_lpc(UART0,"\n***Starting adis-dev test ***\r\n\r\n");

	printf_lpc(UART0,"\n***Board is defined as: %s ***\r\n", infoquery_gfe_boardtag() );

	init_color_led();

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

	adis_init();

	// Testing some ADIS register reads and a write.
	adis_read_id();
	adis_read_gpio_ctl();
	adis_write_smpl_prd(0, 5);
	adis_read_smpl_prd();

	// Wait for all transactions to complete before enabling interrupts.
	util_wait_msecs(2000);

	adis_dev_enable_dio1();

	all_led_off();
	while(1) {
//		color_led_flash(3, RED_LED, FLASH_FAST);
//		util_wait_msecs(1000);
		color_led_flash(2, BLUE_LED, FLASH_SLOW);
		util_wait_msecs(1500);
//		color_led_flash(3, BLUE_LED, FLASH_FAST);
//		util_wait_msecs(1000);
		//adis_process_done_q();
	}

	return(0);
}

