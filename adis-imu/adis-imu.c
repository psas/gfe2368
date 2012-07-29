/*! \file adis-imu.c
 *
 * ADIS sensor with SPI interface
 *
 * Manufacturer: Analog Devices
 * Part Number:  ADIS16405BMLZ0
 *               388
 *               AD091411-1
 * Ref: http://www.analog.com/en/mems-sensors/mems-inertial-sensors/adis16223/products/product.html
 */

#include <stdint.h>

#include "lpc23xx.h"

#include "lpc23xx-pll.h"
#include "lpc23xx-spi.h"
#include "lpc23xx-uart.h"
#include "lpc23xx-util.h"
#include "lpc23xx-vic.h"

#include "printf-lpc.h"

#include "gfe2368-info.h"
#include "gfe2368-util.h"

#include "adis-imu.h"

/*! \brief create a write address
 *
 * @param reg
 * @return 16 bit address, MSBit to 1
 */
//static uint16_t adis_create_register_write_address(adis_regaddr reg, adis_regside side) {
//	uint16_t cmdaddr;
//
//	if(side == ADIS_UPPER) {
//		reg += 1;
//	}
//	cmdaddr = ( (reg | 0b1000) << 8);
//    return cmdaddr;
//}

/*! \brief create a read address
 *
 * @param reg
 * @param side
 * @return 16 bit address MSBit to 0
 */
//static uint16_t adis_imu_create_register_read_address(adis_regaddr reg, adis_regside side) {
//	uint16_t cmdaddr;
//
//	if(side == ADIS_UPPER) {
//		reg += 1;
//	}
//	cmdaddr = ( (reg & ~(0b1000) ) << 8 );
//    return cmdaddr;
//}

/*! \brief Read the product id
 *
 * @return 16 bit product id
 */
//static uint16_t adis_imu_read_product_id() {
//    uint8_t  cmdaddr;
//    uint16_t data = 0;
//
//    cmdaddr = adis_imu_create_register_read_address(ADIS_PRODUCT_ID, ADIS_LOWER);
//
//    SSEL_LOW;
//
//    spi_transact(cmdaddr, SPI_16BITS) ;
//
//    spi_waitSPIF();
//
//    data = S0SPDR ;
//
//    SSEL_HIGH;
//
//    return(data & 0xf);
//}

/*! \brief Reset the ADIS
 */
void adis_imu_reset() {
	ADIS_RST_HIGH;
	ADIS_RST_LOW;

	util_wait_msecs(ADIS_RST_MSECS);

	ADIS_RST_HIGH;
}


/*! \brief Application loop
 */
//void adis_task() {
//
//
//}

int main (void) {
    pllstart_seventytwomhz() ;
    //   pllstart_sixtymhz() ;
    //   pllstart_fourtyeightmhz() ;
    //
    FIO_ENABLE;

    uart0_init_115200() ;

    vic_enableIRQ();
    vic_enableFIQ();

    uart0_putstring_intr("\r\n***Starting UTIL timing test.***\r\n\r\n");

    init_color_led();

    PINSEL_ADIS_RST;
    PINMODE_ADIS_RST_PULLUP;

    printf_lpc(UART0,"\r\n***Starting ADIS test***\r\n");

    color_led_flash(ADIS_IMU_LEDFLASHES, BLUE_LED,  FLASH_NORMAL ) ;

    adis_imu_reset();

    // spi_init_master_MSB_16(CCLK_DIV8, SPI_100KHZ);
    // stat_led_flash_fast(cycles); // initial visual check

    // scp_task() ;

    // stat_led_flash_slow(2);

    printf_lpc(UART0, "\r\n***Done***\r\n");

    return(0);

}

