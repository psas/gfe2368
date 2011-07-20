


/*
 * gfe2368-util.h
 */

#ifndef _GFE2368_UTIL
#define _GFE2368_UTIL

#include "lpc23xx.h"

#define MAX_I2ASTRING    33

#define BLUE_LED_PIN      25

#define STAT_LED_ENABLE  ( FIO1DIR |= (1<<BLUE_LED_PIN) )
#define STAT_LED_DISABLE ( FIO1DIR = FIO1DIR & (~(1<<BLUE_LED_PIN)) )

#define STAT_LED_ON      ( FIO1CLR = (1 << BLUE_LED_PIN) )
#define STAT_LED_OFF     ( FIO1SET = (1 << BLUE_LED_PIN) )

void init_stat_led() ;
void stat_led_flash_slow(uint32_t cycles) ;
void stat_led_flash(uint32_t cycles) ;
void stat_led_flash_fast(uint32_t cycles) ;

#endif
