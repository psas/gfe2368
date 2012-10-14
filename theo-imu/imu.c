
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
    buf[1] = (timestamp & (0xFF<<24))>>24;
    buf[2] = (timestamp & (0xFF<<16))>>16;
    buf[3] = (timestamp & (0xFF<<8))>>8;
    buf[4] = (timestamp & (0xFF<<0))>>0;
    buf[5] = status;
    buf[6] = (x & (0xFF<<8))>>8;
    buf[7] = (x & (0xFF<<0))>>0;
    buf[8] = (y & (0xFF<<8))>>8;
    buf[9] = (y & (0xFF<<0))>>0;
    buf[10] = (z & (0xFF<<8))>>8;
    buf[11] = (z & (0xFF<<0))>>0;
    buf[12] = extra_data;
}

static void LIS331HH_get_data_callback(i2c_master_xact_t* caller,
        i2c_master_xact_t* i2c_s)
{
    //TODO: send xact_success failure over USB
    if(!(i2c_s->xact_success)){
        uart0_putstring("\n***LIS331HH GET DATA FAILED***\n");
        return;
    }

    uint8_t buf[IMU_PACKET_LENGTH];
    fill_imu_packet(buf, ADDR_ACC, LIS331HH_timestamp, i2c_s->i2c_rd_buffer[0],
            (uint16_t)i2c_s->i2c_rd_buffer[1] | (uint16_t)i2c_s->i2c_rd_buffer[2] << 8,
            (uint16_t)i2c_s->i2c_rd_buffer[3] | (uint16_t)i2c_s->i2c_rd_buffer[4] << 8,
            (uint16_t)i2c_s->i2c_rd_buffer[5] | (uint16_t)i2c_s->i2c_rd_buffer[6] << 8,
            0);
    int bytes_sent = USBHwEPWrite (ACC_EP, buf, IMU_PACKET_LENGTH);
    if(bytes_sent != IMU_PACKET_LENGTH){
        uart0_putstring("\n***LIS331HH WRITE DATA FAILED***\n");
    }
}

static void L3G4200D_get_data_callback(i2c_master_xact_t* caller,
        i2c_master_xact_t* i2c_s)
{
    //TODO: send xact_success failure over USB
    if(!(i2c_s->xact_success)){
        uart0_putstring("\n***L3G4200D GET DATA FAILED***\n");
        return;
    }
    uint8_t buf[IMU_PACKET_LENGTH];
    fill_imu_packet(buf, ADDR_GYR, L3G4200D_timestamp, i2c_s->i2c_rd_buffer[1],
            (uint16_t)i2c_s->i2c_rd_buffer[2] | (uint16_t)i2c_s->i2c_rd_buffer[3] << 8,
            (uint16_t)i2c_s->i2c_rd_buffer[4] | (uint16_t)i2c_s->i2c_rd_buffer[5] << 8,
            (uint16_t)i2c_s->i2c_rd_buffer[6] | (uint16_t)i2c_s->i2c_rd_buffer[7] << 8,
            i2c_s->i2c_rd_buffer[0]);//temperature data);

    int bytes_sent = USBHwEPWrite (GYR_EP, buf, IMU_PACKET_LENGTH);
    if(bytes_sent != IMU_PACKET_LENGTH){
        uart0_putstring("\n***L3G4200D WRITE DATA FAILED***\n");
    }
}

static void LSM303DLH_m_get_data_callback(i2c_master_xact_t* caller,
        i2c_master_xact_t* i2c_s)
{
    //TODO: send xact_success failure over USB
    if(!(i2c_s->xact_success)){
        uart0_putstring("\n***LSM303DLH_m GET DATA FAILED***\n");
        return;
    }
    uint8_t buf[IMU_PACKET_LENGTH];
    fill_imu_packet(buf, ADDR_MAG, LSM303DLH_m_timestamp, 0,
            (uint16_t)i2c_s->i2c_rd_buffer[0] | (uint16_t)i2c_s->i2c_rd_buffer[1] << 8,
            (uint16_t)i2c_s->i2c_rd_buffer[2] | (uint16_t)i2c_s->i2c_rd_buffer[3] << 8,
            (uint16_t)i2c_s->i2c_rd_buffer[4] | (uint16_t)i2c_s->i2c_rd_buffer[5] << 8,
            0);//temperature data);

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
        if(LSM303DLH_M_STUCK){
            LSM303DLH_m_get_data(NULL);
        }
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
//	DISABLE_GPIO_INT;
    uint32_t timestamp = T0TC;
    //check IOIntStatus bit 0 - LPCUM 10.5.6.1. If 1, PORT0 interrupt.
//  if(!(IOIntStat & 1)){
////    ENABLE_GPIO_INT;
//      return; //interrupt is not for IMU
//  }

    //IO0IntStatR for pin interrupt status
    //IO0IntClr to clear interrupt
    if((IO0IntStatR & GYRO_INT2) || L3G4200D_STUCK){
        L3G4200D_timestamp = timestamp;
        L3G4200D_get_data(L3G4200D_get_data_callback);
        IO0IntClr = GYRO_INT2;
    }

    if((IO0IntStatR & ACCEL_INT1) || LIS331HH_STUCK){
        LIS331HH_timestamp = timestamp;
        LIS331HH_get_data(LIS331HH_get_data_callback);
        IO0IntClr =	ACCEL_INT1;
    }

    if((IO0IntStatR & MAG_DRDY) || LSM303DLH_M_STUCK){
        LSM303DLH_m_timestamp = timestamp;
        LSM303DLH_m_get_data(LSM303DLH_m_get_data_callback);
        IO0IntClr =	 MAG_DRDY;
    }

    EXIT_INTERRUPT;
}

static void poll_wait(i2c_iface i2c_ch) {
    switch(i2c_ch){
    case I2C0:
        while(is_binsem_locked(&i2c0_binsem_g)== 1);
        break;
    case I2C1:
        while(is_binsem_locked(&i2c1_binsem_g)== 1);
        break;
    case I2C2:
        while(is_binsem_locked(&i2c2_binsem_g)== 1);
        break;
     }
}


static void imu_init(void){
    i2c_init(I2C0, DEFAULT);
    i2c_kHz(I2C0, 400);

    i2c_init(I2C1, I2C1_ALTPIN);
    i2c_kHz(I2C1, 400);

    i2c_init(I2C2, DEFAULT);
    i2c_kHz(I2C2, 400);

    //configure other wires, (addresses, etc.)
    FIO0DIR |= ACCEL_SA0 | ACCEL_CS | MAG_SA | GYRO_SA0 | GYRO_CS;
    FIO0SET = ACCEL_CS | GYRO_CS;
    FIO0CLR = ACCEL_SA0 | MAG_SA | GYRO_SA0;

    L3G4200D_init(I2C0);
    LIS331HH_init(I2C1);
    LSM303DLH_init_m(I2C2);

    poll_wait(I2C0);
    poll_wait(I2C1);
    poll_wait(I2C2);

    timer_init(TIMER_0, 0x0 , CCLK_DIV1);
    RESET_TIMER0;
    START_TIMER0;

    VIC_SET_EINT3_GPIO_HANDLER(imu_isr);
    ENABLE_INT(VIC_EINT3_GPIO);
}

int main(void){

    uint8_t abClassReqData[8];

    pllstart_seventytwomhz();
    mam_enable();
    uart0_init_115200();
    init_color_led();

    uart0_putstring("\n\n***Starting IMU***\n\n");

    USBInit();
    USBRegisterRequestHandler(REQTYPE_TYPE_VENDOR, imu_ctrl, abClassReqData);
    USBHwRegisterEPIntHandler(ACC_EP, NULL);
    USBHwRegisterEPIntHandler(GYR_EP, NULL);
    USBHwRegisterEPIntHandler(MAG_EP, NULL);
    USBHwRegisterEPIntHandler(CAC_EP, NULL);
    USBHwConnect(true);

    cycle_led();
    color_led_flash(5, BLUE_LED, FLASH_FAST );
    RED_LED_ON;
    imu_init();

    while(1){
        IDLE_MODE;
    }

    return 0;
}

