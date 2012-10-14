/*
 * sys-mgr.c
 *
 * Manages avionics system level stuff so that the flight computer doesn't have
 * to do it. Implements the following state machine:
 *
 *
                       Off
                       ^ |(Power on)
                       | V
            (power off)| Manage the battery charger
                       | |
                       | V
                      sleep
                       ^ |(Serial activity)
(serial activity stops)| V
                Look for FC_ON_CMD
                       ^ |(FC_ON_CMD)
           (FC_CMD_OFF)| V
                  GPIO USB mode
*/


/* GPIO USB command format:
 * action|register|[value]
 * action: 2 bits, perform read/write/update of register with value
 * register: 4 bytes, the address of the register to access
 * value: 4 bytes, the data to perform write or update with.
 */



#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "lpc23xx.h"
#include "lpc23xx-pll.h"
#include "lpc23xx-mam.h"
#include "lpc23xx-power.h"
#include "lpc23xx-uart.h"
#include "gfe2368-util.h"
#include "BQ24725.h"
#include "lpc23xx-i2c.h"
#include "lpc23xx-usb.h"
#include "lpc23xx-util.h"
#include "aps.h"
#include "printf-lpc.h"

typedef enum sys_mgr_state {INIT, SLEEP, SERIAL, GPIO_USB_INIT, GPIO_USB} sys_mgr_state;

static sys_mgr_state state = INIT;


bool find_cmd(char * buffer, char * cmd){// input list?
	int cmd_index = 0;
	int buf_index = 0;
	while(buffer[buf_index] != '\n'){
	    if(buffer[buf_index++] == cmd[cmd_index]){
	        if(cmd[++cmd_index] == '\0'){
	            return true;
	        }
		}else{
		    if(cmd_index != 0){
                cmd_index = 0;
                --buf_index;
		    }
		}
	}
	return false;
}

void serial_cb(char * buffer){
    if(find_cmd(buffer, "TURNTHEHELLON")){
        state = GPIO_USB_INIT;
        FIO0SET = (1<<FC_SPS_PIN);
    }else if(find_cmd(buffer, "WHYAREYOUNOTOFFYET")){
        //USBHwConnect(false);
        state = SLEEP;
        FIO0CLR = (1<<FC_SPS_PIN);
    }
}

void sys_mgr_sleep(){
    IO0IntEnR |= (0x1<<3);
    IO0IntClr |= (0x1<<3);
    IDLE_MODE;
//    while(!(IO0IntStatR & 1<<3));// imitation sleep. There's a problem in
    //pllstart that causes everything to omgwtfbbq after a sleep mode
    //safe_sleep(GPIO0WAKE);
    //Freq saved_freq = pllquery_cclk_mhz();
    //INTWAKE = 0x1<<7;
    //SLEEP_MODE;
    //pllstart(saved_freq);
    IO0IntClr |= (0x1<<3);
    IO0IntEnR &= ~(0x1<<3);
}

void mainloop(){
	while(1){
	    color_led_flash(5, RED_LED, FLASH_FAST);
		switch(state){
		case SLEEP:
		    sys_mgr_sleep();
			break;
		case SERIAL:
			break;
        case GPIO_USB_INIT:
            uart0_putstring("\nENTERED GPIO_USB MODE\n");

//            USBHwConnect(true);
            state = GPIO_USB;
            break;
		case GPIO_USB:
			break;
		default:
			//todo: see if we can determine the state by looking at gpio settings
			break;
		}
	}
}

bool gpio_request(TSetupPacket *pSetup, int *piLen, uint8_t **ppbData){
    uint8_t * gpio_reg = *ppbData;
    uint8_t * val_buf = *ppbData;
    uint32_t val;
    //int left;
    if(REQTYPE_GET_DIR(pSetup->bmRequestType)){
        //device-to-host
        printf_lpc(UART0, "%x\n%x\n%x\n%x\n%x\n", FIO0PIN, FIO1PIN, FIO2PIN, FIO3PIN, FIO4PIN);
        gpio_reg[0] = (FIO0PIN & 0xFF<<0)>>0;
        gpio_reg[1] = (FIO0PIN & 0xFF<<8)>>8;
        gpio_reg[2] = (FIO0PIN & 0xFF<<16)>>16;
        gpio_reg[3] = (FIO0PIN & 0xFF<<24)>>24;
        gpio_reg[4] = (FIO1PIN & 0xFF<<0)>>0;
        gpio_reg[5] = (FIO1PIN & 0xFF<<8)>>8;
        gpio_reg[6] = (FIO1PIN & 0xFF<<16)>>16;
        gpio_reg[7] = (FIO1PIN & 0xFF<<24)>>24;
        gpio_reg[8] = (FIO2PIN & 0xFF<<0)>>0;
        gpio_reg[9] = (FIO2PIN & 0xFF<<8)>>8;
        gpio_reg[10] = (FIO2PIN & 0xFF<<16)>>16;
        gpio_reg[11] = (FIO2PIN & 0xFF<<24)>>24;
        gpio_reg[12] = (FIO3PIN & 0xFF<<0)>>0;
        gpio_reg[13] = (FIO3PIN & 0xFF<<8)>>8;
        gpio_reg[14] = (FIO3PIN & 0xFF<<16)>>16;
        gpio_reg[15] = (FIO3PIN & 0xFF<<24)>>24;
        gpio_reg[16] = (FIO4PIN & 0xFF<<0)>>0;
        gpio_reg[17] = (FIO4PIN & 0xFF<<8)>>8;
        gpio_reg[18] = (FIO4PIN & 0xFF<<16)>>16;
        gpio_reg[19] = (FIO4PIN & 0xFF<<24)>>24;
        *piLen = 4*5;

    }else{
        //host-to-device

        val = (val_buf[0]) | (val_buf[1]<<8) | (val_buf[2]<<16) | (val_buf[3]<<24);
        if(pSetup->bRequest & 0x80){
            uart0_putstring("Setting GPIO\n");
            switch(pSetup->bRequest & 0x0F){
            case 0:
                FIO0SET = val;
                break;
            case 1:
                FIO1SET = val;
                break;
            case 2:
                FIO2SET = val;
                break;
            case 3:
                FIO3SET = val;
                break;
            case 4:
                FIO4SET = val;
                break;
            default:
                break;
            }
        }else{
            uart0_putstring("Clearing GPIO\n");
            switch(pSetup->bRequest & 0x0F){
            case 0:
                FIO0CLR = val;
                break;
            case 1:
                FIO1CLR = val;
                break;
            case 2:
                FIO2CLR = val;
                break;
            case 3:
                FIO3CLR = val;
                break;
            case 4:
                FIO4CLR = val;
                break;
            default:
                break;
            }
        }
    }
    return true;
}


void settings_BQ(){//todo: verify acok after each step
    uart0_putstring("Setting BQ\n");
    BQ24725_charge_options BQ24725_rocket_init = {
                .ACOK_deglitch_time = t150ms,
                .WATCHDOG_timer = disabled,
                .BAT_depletion_threshold = FT70_97pct,
                .EMI_sw_freq_adj = dec18pct,
                .EMI_sw_freq_adj_en = sw_freq_adj_disable,
                .IFAULT_HI_threshold = l700mV,
                .LEARN_en = LEARN_disable,
                .IOUT = adapter_current,
                .ACOC_threshold = l1_66X,
                .charge_inhibit = charge_enable
            };

            BQ24725_SetChargeOption(&BQ24725_rocket_init);
            uart0_putstring("Setting current\n");
            BQ24725_SetChargeCurrent(0x400);
            uart0_putstring("Setting voltage\n");
            BQ24725_SetChargeVoltage(0x41A0);
            uart0_putstring("Setting input\n");
            BQ24725_SetInputCurrent(0x1000);
}

void GPIO_isr(void){
    if(IO2IntStatR & (1<<ACOK_PIN)){
        settings_BQ();
        IO2IntClr &= ~(1<<ACOK_PIN);
        IO2IntEnR &= ~(1<<ACOK_PIN);

        DISABLE_INT(VIC_EINT3_GPIO);
    }
    EXIT_INTERRUPT;
}

int main(){
    uint8_t abClassReqData[64];
    FIO0DIR = (1<<FC_SPS_PIN) | (1<<ATV_SPS_PIN) | (1<<RC_POWER_PIN) |
              (1<<ROCKET_READY_PIN) | (1<<WIFI_POWER_PIN) | (1<<RC_TETHER);
    PINMODE1 = (0x2<<(2*(IOUT_PIN-16)));
//    PINMODE4 = (0x2<<(2*ACOK_PIN));
    PCONP = 0;
    pllstart_seventytwomhz();
    mam_enable();
    uart0_init_115200();
    init_color_led();
	all_led_off();
	cycle_led();
	uart0_putstring("\nStarting apc\n");
	uart0_set_getstring_cb(serial_cb);
	//usb_init
//	BQ24725_init(I2C2, DEFAULT);
//	//printf_lpc(UART0, "FIO2: %x\n", FIO2PIN);
//	BQ24725_GetManufactureID(NULL);
//	if(FIO2PIN & (1<<ACOK_PIN)){
//	    uart0_putstring("ACOK set on startup\n");
//	    settings_BQ();
//	}else{
//	    uart0_putstring("ACOK not set on startup\n");
//        IO2IntEnR |= 1<<ACOK_PIN;
//        VIC_SET_EINT3_GPIO_HANDLER(GPIO_isr);
//        ENABLE_INT(VIC_EINT3_GPIO);
//	}
	USBInit(abDescriptors);
	USBRegisterRequestHandler(REQTYPE_TYPE_VENDOR, gpio_request, abClassReqData);
    VIC_SET_USB_PRIORITY(1);
	VIC_SET_USB_HANDLER(USBIntHandler);
	ENABLE_INT(VIC_USB);
	USBHwConnect(true);
	state = SLEEP;
	uart0_putstring("Entering mainloop\n");
	mainloop();
	return 0;
}
