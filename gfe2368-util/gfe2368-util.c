
/*
 * gfe2368-util.c
 */

#include <stdint.h>
#include <string.h>

#include "lpc23xx.h"

#include "lpc23xx-pll.h"
#include "lpc23xx-uart.h"
#include "lpc23xx-util.h"

#include "gfe2368-util.h"

/*
 * init_stat_led
 * call this to initialize PINSEL register and FIO
 */
void init_color_led() {
    FIO_ENABLE;

    RED_LED_ENABLE;
    GREEN_LED_ENABLE;
    BLUE_LED_ENABLE;

    RED_LED_OFF;
    GREEN_LED_OFF;
    BLUE_LED_OFF;
}

/*
 * color_led_flash
 */
void color_led_flash(uint32_t cycles, 
        ledcolor_type   led, 
        flashspeed_type speed ) {

    int x              = 0;
    int interval       = 100000;
    int scale          = 1;

    Freq cclk;
    cclk  =  pllquery_cclk_mhz();

    switch(speed) {
        case FLASH_SLOW:
            scale = 0.1;
            break;
        case FLASH_NORMAL:
            scale = 0.01; 
            break;
        case FLASH_FAST:
            scale = 0.003;
            break;
        default:
            scale = 0.01;
            break;
    }
    switch(cclk) {
        case ZERO:
            interval = 100000;
            break;
        case FOURTY_EIGHT_MHZ:
            interval = scale * 48000000;
            break;
        case SIXTY_MHZ:
            interval = scale * 60000000;
            break;
        case SEVENTY_TWO_MHZ:
            interval = scale * 72000000;
            break;
        default:
            interval = 100000;
            break;
    }

    if(cycles > 0) {
        for(;;) {
            if(cycles==0) break;
            x++;
            if (x == interval) {
                //  uart0_putstring(".\n");
                --cycles;
                switch(led) {
                    case RED_LED:
                        RED_LED_ON;
                        break;
                    case BLUE_LED:
                        BLUE_LED_ON;
                        break;
                    case GREEN_LED:
                        GREEN_LED_ON;
                        break;
                    default:
                        break;
                }
            } else if (x >= (interval * 2)) {
                switch(led) {
                    case RED_LED:
                        RED_LED_OFF;
                        break;
                    case BLUE_LED:
                        BLUE_LED_OFF;
                        break;
                    case GREEN_LED:
                        GREEN_LED_OFF;
                        break;
                    default:
                        break;
                }
                x = 0;
            }
        }
    }
}


