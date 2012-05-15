
/*
 * led-test.c
 */

#include <limits.h>
#include <stdint.h>

#include "lpc23xx.h"

#include "lpc23xx-pll.h"
#include "lpc23xx-uart.h"
#include "lpc23xx-util.h"
#include "printf-lpc.h"

#include "gfe2368-info.h"
#include "gfe2368-util.h"
#include "led-test.h"

int main (void) {

    int32_t cycles = 3;

 //   pllstart_seventytwomhz() ;
    pllstart_sixtymhz() ;
 //   pllstart_fourtyeightmhz() ;

    info_init();

    uart0_init_115200() ;

    uart0_putstring("\n***Starting gfe color led test***\n\n");
    
    uart0_putstring("\n***Board is defined as: ");

    uart0_putstring( infoquery_gfe_boardtag() );

    uart0_putstring(" ***\n");

    init_color_led();

    RED_LED_OFF;
    BLUE_LED_OFF;
    GREEN_LED_OFF;

    printf_lpc(UART0,"5 FAST flashes...red, blue then green\n"); 
    color_led_flash(5, RED_LED, FLASH_FAST ) ;
    RED_LED_OFF;
    color_led_flash(5, BLUE_LED,  FLASH_FAST ) ;
    BLUE_LED_OFF;
    color_led_flash(5, GREEN_LED, FLASH_FAST ) ;
    GREEN_LED_OFF;

    printf_lpc(UART0,"5 NORMAL flashes...red, blue then green\n"); 
    color_led_flash(5, RED_LED, FLASH_NORMAL ) ;
    RED_LED_OFF;
    color_led_flash(5, BLUE_LED,  FLASH_NORMAL ) ;
    BLUE_LED_OFF;
    color_led_flash(5, GREEN_LED, FLASH_NORMAL ) ;
    GREEN_LED_OFF;

    printf_lpc(UART0,"2 SLOW flashes...red, blue then green\n"); 
    color_led_flash(2, RED_LED, FLASH_SLOW ) ;
    RED_LED_OFF;
    color_led_flash(2, BLUE_LED,  FLASH_SLOW ) ;
    BLUE_LED_OFF;
    color_led_flash(2, GREEN_LED, FLASH_SLOW ) ;
    GREEN_LED_OFF;

    printf_lpc(UART0,"red...");
    RED_LED_ON;
    util_wait_msecs(2000) ;
    RED_LED_OFF;

    printf_lpc(UART0,"blue...");
    BLUE_LED_ON;
    util_wait_msecs(2000) ;
    BLUE_LED_OFF;

    printf_lpc(UART0,"green...");
    GREEN_LED_ON;
    util_wait_msecs(2000) ;
    GREEN_LED_OFF;


    // negative numbers in itoa
    uart0_putstring("Print a negative number: ");
    uart0_putstring(util_itoa(-42, 10));
    uart0_putstring("\n");

    uart0_putstring("Print a negative number: ");
    uart0_putstring(util_itoa(-42, 16));
    uart0_putstring("\n");

    uart0_putstring("0b");
    uart0_putstring(util_itoa(cycles,2));
    uart0_putstring(" cycles.\n");

    uart0_putstring("0d");
    uart0_putstring(util_itoa(cycles,10));
    uart0_putstring(" cycles.\n");

    uart0_putstring("0x");
    uart0_putstring(util_itoa(cycles,16));
    uart0_putstring(" cycles.\n");

    uart0_putstring("\n\n***Done***\n\n");

    while(1) {
        printf_lpc(UART0,"2 SLOW flashes...red, blue then green\n"); 
        color_led_flash(2, RED_LED, FLASH_SLOW ) ;
        RED_LED_OFF;
        color_led_flash(2, BLUE_LED,  FLASH_SLOW ) ;
        BLUE_LED_OFF;
        color_led_flash(2, GREEN_LED, FLASH_SLOW ) ;
        GREEN_LED_OFF;
    }


    return(0);

}



