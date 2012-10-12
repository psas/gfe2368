


/*
 * gfe2368-util.h
 */

#ifndef _GFE2368_UTIL
#define _GFE2368_UTIL

#include <stdint.h>
#include "lpc23xx.h"

#define MAX_I2ASTRING        33

#define RED_LED_PIN          19
#define BLUE_LED_PIN         22
#define GREEN_LED_PIN        25

#define RED_LED_ENABLE       ( FIO1DIR |= (1 << RED_LED_PIN) )
#define RED_LED_DISABLE      ( FIO1DIR = FIO1DIR & (~(1<<RED_LED_PIN)) )

#define BLUE_LED_ENABLE      ( FIO1DIR |= (1 << BLUE_LED_PIN) )
#define BLUE_LED_DISABLE     ( FIO1DIR = FIO1DIR & (~(1<<BLUE_LED_PIN)) )

#define GREEN_LED_ENABLE      ( FIO1DIR |= (1 << GREEN_LED_PIN) )
#define GREEN_LED_DISABLE     ( FIO1DIR = FIO1DIR & (~(1<<GREEN_LED_PIN)) )

#define RED_LED_ON           ( FIO1SET = (1 << RED_LED_PIN) )
#define RED_LED_OFF          ( FIO1CLR = (1 << RED_LED_PIN) )

#define BLUE_LED_ON          ( FIO1SET = (1 << BLUE_LED_PIN) )
#define BLUE_LED_OFF         ( FIO1CLR = (1 << BLUE_LED_PIN) )

#define GREEN_LED_ON         ( FIO1SET = (1 << GREEN_LED_PIN) )
#define GREEN_LED_OFF        ( FIO1CLR = (1 << GREEN_LED_PIN) )

typedef enum {FLASH_SLOW=0, FLASH_NORMAL, FLASH_FAST} flashspeed_type;
typedef enum {RED_LED=0, BLUE_LED, GREEN_LED} ledcolor_type;

void init_color_led() ;
void all_led_off() ;
void cycle_led() ;
void color_led_flash(uint32_t cycles, ledcolor_type led, flashspeed_type speed ) ;


#endif
