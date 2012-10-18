
/*
 * imu.c
 *
 * Manages the sensors that make up theo-imu and sends the data over USB
 */

#include <stdint.h>

#include "lpc23xx.h"
#include "lpc23xx-mam.h"
#include "lpc23xx-pll.h"
#include "lpc23xx-vic.h"
#include "lpc23xx-power.h"
#include "lpc23xx-i2c.h"
#include "lpc23xx-timer.h"
#include "lpc23xx-binsem.h"
#include "lpc23xx-usb.h"
#include "lpc23xx-uart.h"
#include "lpc23xx-util.h"
#include "gfe2368-util.h"
#include "printf-lpc.h"

#include "imu.h"
#include "L3G4200D.h"
#include "LIS331HH.h"
#include "LSM303DLH.h"

static uint32_t L3G4200D_timestamp;
static uint32_t LIS331HH_timestamp;
static uint32_t LSM303DLH_m_timestamp;

static inline void fill_imu_packet(uint8_t * buf, uint8_t ID,
        uint32_t timestamp, uint8_t status, uint16_t x, uint16_t y, uint16_t z,
        uint8_t extra_data)
{
    buf[0] = ID;
    buf[1] = timestamp >> 24;
    buf[2] = timestamp >> 16;
    buf[3] = timestamp >> 8;
    buf[4] = timestamp;
    buf[5] = status;
    buf[6] = x >> 8;
    buf[7] = x;
    buf[8] = y >> 8;
    buf[9] = y;
    buf[10] = z >> 8;
    buf[11] = z;
    buf[12] = extra_data;
}

static void LIS331HH_get_data_callback(i2c_master_xact_t* i2c_s)
{
    //TODO: send failure over USB
    switch(i2c_s->state){
    case I2C_ERROR:
        uart0_putstring("\n***LIS331HH I2C call failed***\n");
        break;
    case I2C_SLAVE_NOTACK:
        uart0_putstring("\n***LIS331HH NACKed in response to I2C***\n");
        break;
    default:
        break;
    }
    uart0_putstring("ACC:");
    uart0_putstring(util_uitoa(i2c_s->rd_buffer[1] | (i2c_s->rd_buffer[2] << 8),DEC));
    uart0_putstring("\n");
    uint8_t buf[IMU_PACKET_LENGTH];
    fill_imu_packet(buf, ADDR_ACC, LIS331HH_timestamp, i2c_s->rd_buffer[0],
            i2c_s->rd_buffer[1] | (i2c_s->rd_buffer[2] << 8),
            i2c_s->rd_buffer[3] | (i2c_s->rd_buffer[4] << 8),
            i2c_s->rd_buffer[5] | (i2c_s->rd_buffer[6] << 8),
            0);
    int bytes_sent = USBHwEPWrite (ACC_EP, buf, IMU_PACKET_LENGTH);
    if(bytes_sent != IMU_PACKET_LENGTH){
        uart0_putstring("\n***LIS331HH WRITE DATA FAILED***\n");
    }
}

static void L3G4200D_get_data_callback(i2c_master_xact_t* i2c_s){
    //TODO: send xact_success failure over USB
    switch(i2c_s->state){
    case I2C_ERROR:
        uart0_putstring("\n***L3G4200D I2C call failed***\n");
        break;
    case I2C_SLAVE_NOTACK:
        uart0_putstring("\n***L3G4200D NACKed in response to I2C***\n");
        break;
    default:
        break;
    }

//    uart0_putstring("GYR:");
//    uart0_putstring(util_uitoa(i2c_s->rd_buffer[2] | (i2c_s->rd_buffer[3] << 8),DEC));
//    uart0_putstring("\n");
    uint8_t buf[IMU_PACKET_LENGTH];
    fill_imu_packet(buf, ADDR_GYR, L3G4200D_timestamp, i2c_s->rd_buffer[1],
            i2c_s->rd_buffer[2] | (i2c_s->rd_buffer[3] << 8),
            i2c_s->rd_buffer[4] | (i2c_s->rd_buffer[5] << 8),
            i2c_s->rd_buffer[6] | (i2c_s->rd_buffer[7] << 8),
            i2c_s->rd_buffer[0]);//temperature data

    int bytes_sent = USBHwEPWrite (GYR_EP, buf, IMU_PACKET_LENGTH);
    if(bytes_sent != IMU_PACKET_LENGTH){
        uart0_putstring("\n***L3G4200D WRITE DATA FAILED***\n");
    }
}

static void LSM303DLH_m_get_data_callback(i2c_master_xact_t* i2c_s){
    //TODO: send xact_success failure over USB
    switch(i2c_s->state){
    case I2C_ERROR:
        uart0_putstring("\n***LSM303DLH I2C call failed***\n");
        break;
    case I2C_SLAVE_NOTACK:
        uart0_putstring("\n***LSM303DLH NACKed in response to I2C***\n");
        break;
    default:
        break;
    }
//    uart0_putstring("MAG:");
//    uart0_putstring(util_uitoa(i2c_s->rd_buffer[0] | (i2c_s->rd_buffer[1] << 8),DEC));
//    uart0_putstring("\n");
//
//    color_led_flash(1, GREEN_LED, FLASH_FAST);
    uint8_t buf[IMU_PACKET_LENGTH];
    fill_imu_packet(buf, ADDR_MAG, LSM303DLH_m_timestamp, 0,
            i2c_s->rd_buffer[0] | (i2c_s->rd_buffer[1] << 8),
            i2c_s->rd_buffer[2] | (i2c_s->rd_buffer[3] << 8),
            i2c_s->rd_buffer[4] | (i2c_s->rd_buffer[5] << 8),
            0);
    int bytes_sent = USBHwEPWrite (MAG_EP, buf, IMU_PACKET_LENGTH);
    if(bytes_sent != IMU_PACKET_LENGTH){
        uart0_putstring("\n***LSM303DLH_m WRITE DATA FAILED***\n");
    }

}

static bool imu_ctrl(TSetupPacket *pSetup, int *piLen, uint8_t **ppbData){
    //todo: set sensors to idle/power_down on INST_STOP
    switch(IMU_INST(pSetup->bRequest)){
    case INST_RESET:
        break;
    case INST_GO:
        //IO0IntEnR |= MAG_INT1 | MAG_INT1;
        IO0IntEnR |= MAG_DRDY;
        IO0IntEnR |= ACCEL_INT1;
        IO0IntEnR |= GYRO_INT2;
        if(L3G4200D_STUCK){
            L3G4200D_get_data(NULL);
        }
        if(LIS331HH_STUCK){
            LIS331HH_get_data(NULL);
        }
        GREEN_LED_ON;
        break;
    case INST_STOP:
//      IO0IntEnR &= ~(MAG_INT1 | MAG_INT1);
        IO0IntEnR &= ~(MAG_DRDY);
        IO0IntEnR &= ~(ACCEL_INT1);
        IO0IntEnR &= ~(GYRO_INT2);
        break;
    case INST_SET_SPEED:
        break;
    default:
        break;
    }
    return true;
}

void imu_isr(void){
    uint32_t timestamp = T0TC;

    //IO0IntStatR for pin interrupt status
    //IO0IntClr to clear interrupt
    if((IO0IntStatR & GYRO_INT2) || L3G4200D_STUCK){
        //uart0_putstring("L3G\n");
        L3G4200D_timestamp = timestamp;
        L3G4200D_get_data(L3G4200D_get_data_callback);
        IO0IntClr = GYRO_INT2;
    }

    if((IO0IntStatR & ACCEL_INT1) || LIS331HH_STUCK){
        //uart0_putstring("LIS\n");
        LIS331HH_timestamp = timestamp;
        LIS331HH_get_data(LIS331HH_get_data_callback);
        IO0IntClr =	ACCEL_INT1;
    }

    if(IO0IntStatR & MAG_DRDY){
        //uart0_putstring("LSM\n");
        LSM303DLH_m_timestamp = timestamp;
        LSM303DLH_m_get_data(LSM303DLH_m_get_data_callback);
        IO0IntClr =	 MAG_DRDY;
    }
    EXIT_INTERRUPT;
}

static void imu_init(void){
    i2c_init(I2C0, DEFAULT);
    i2c_kHz(I2C0, 400);

    i2c_init(I2C1, I2C1_ALTPIN);
    i2c_kHz(I2C1, 400);

    i2c_init(I2C2, DEFAULT);
    i2c_kHz(I2C2, 400);
    uart0_putstring("past i2c init\n");
    //configure other wires, (addresses, etc.)
    FIO0DIR |= ACCEL_SA0 | ACCEL_CS | MAG_SA | GYRO_SA0 | GYRO_CS;
    FIO0SET = ACCEL_CS | GYRO_CS;
    FIO0CLR = ACCEL_SA0 | MAG_SA | GYRO_SA0;

    L3G4200D_init(I2C0);
    LIS331HH_init(I2C1);
    LSM303DLH_init_m(I2C2);

    uart0_putstring("past sensor init\n");

//    poll_wait(I2C0);
//    uart0_putstring("past poll_wait 0\n");
    poll_wait(I2C1);
    uart0_putstring("past poll_wait 1\n");
    poll_wait(I2C2);
    uart0_putstring("past poll_wait 2\n");

    timer_init(TIMER_0, 0x0 , CCLK_DIV1);
    RESET_TIMER0;
    START_TIMER0;

    VIC_SET_EINT3_GPIO_HANDLER(imu_isr);
    ENABLE_INT(VIC_EINT3_GPIO);

//    IO0IntEnR |= MAG_DRDY;
////    IO0IntEnR |= ACCEL_INT1;
//    IO0IntEnR |= GYRO_INT2;
//    if(L3G4200D_STUCK){
//        L3G4200D_get_data(NULL);
//    }
//    if(LIS331HH_STUCK){
//        LIS331HH_get_data(NULL);
//    }
}

int main(void){



    PCONP = 0; //shut down all unused peripherals

    pllstart_seventytwomhz();
    mam_enable();
    uart0_init_115200();
    init_color_led();

    uart0_putstring("\n\n***Starting IMU***\n\n");

    USBInit(imu_descriptor);
    uint8_t abClassReqData[MAX_PACKET_SIZE0];
    USBRegisterRequestHandler(REQTYPE_TYPE_VENDOR, imu_ctrl, abClassReqData);
    USBHwConnect(true);

    cycle_led();
    imu_init();

    while(1){
        color_led_flash(5, RED_LED, FLASH_FAST);
//        uart0_putstring(util_uitoa(FIO0PIN, HEX));
        if(L3G4200D_STUCK){
            uart0_putstring(" L3G STUCK ");
            L3G4200D_get_data(NULL);
        }
        if(LIS331HH_STUCK){
            uart0_putstring(" LIS STUCK ");
            LIS331HH_get_data(NULL);
        }
//        uart0_putstring("\n");
        //IDLE_MODE;
    }

    return 0;
}

