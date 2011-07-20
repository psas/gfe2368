
/*
 * led-test.h
 */


#ifndef _LED_TEST_H
#define _LED_TEST_H
#define P0_0_ENABLE     ( FIO0DIR |= (1<<0) )
#define P0_0_DISABLE    ( FIO0DIR = FIO0DIR & (~(1<<0)) )
#define P0_1_ENABLE     ( FIO0DIR |= (1<<1) )
#define P0_1_DISABLE    ( FIO0DIR = FIO0DIR & (~(1<<1)) )


#define P0_25_ENABLE     ( FIO0DIR |= (1<<25) )
#define P0_25_DISABLE    ( FIO0DIR = FIO0DIR & (~(1<<25)) )

#define P0_28_ENABLE     ( FIO0DIR |= (1<<28) )
#define P0_28_DISABLE    ( FIO0DIR = FIO0DIR & (~(1<<28)) )
#define P0_27_ENABLE     ( FIO0DIR |= (1<<27) )
#define P0_27_DISABLE    ( FIO0DIR = FIO0DIR & (~(1<<27)) )

#define P0_0_OFF        ( FIO0CLR = (1 << 0) )
#define P0_0_ON         ( FIO0SET = (1 << 0) )
#define P0_1_OFF        ( FIO0CLR = (1 << 1) )
#define P0_1_ON         ( FIO0SET = (1 << 1) )

#define P0_25_OFF        ( FIO0CLR = (1 << 25) )
#define P0_25_ON         ( FIO0SET = (1 << 25) )

#define P0_28_OFF        ( FIO0CLR = (1 << 28) )
#define P0_28_ON         ( FIO0SET = (1 << 28) )
#define P0_27_OFF        ( FIO0CLR = (1 << 27) )
#define P0_27_ON         ( FIO0SET = (1 << 27) )


#endif

