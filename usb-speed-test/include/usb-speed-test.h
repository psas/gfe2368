

#ifndef USB_SPEED_TEST_H_
#define USB_SPEED_TEST_H_

#include "lpc23xx-types.h"

static void GPIO_isr(void)      __attribute__ ((interrupt("IRQ")));
static void USBIntHandler(void) __attribute__ ((interrupt("IRQ")));

#define VIC_GPIO_BIT        (17)
#define ENABLE_GPIO_INT     (VICIntEnable |= (1<<VIC_GPIO_BIT))
#define DISABLE_GPIO_INT    (VICIntEnClr   = (1<<VIC_GPIO_BIT))
#define RAISE_GPIO_INT      (VICSoftInt   |= (1<<VIC_GPIO_BIT))
#define CLR_SW_GPIO_INT     (VICSoftIntClr = (1<<VIC_GPIO_BIT))

// data structure for GET_LINE_CODING / SET_LINE_CODING class requests
typedef struct {
    uint32_t            dwDTERate;
    uint8_t             bCharFormat;
    uint8_t             bParityType;
    uint8_t             bDataBits;
} TLineCoding;

#endif /* USB_SPEED_TEST_H_ */
