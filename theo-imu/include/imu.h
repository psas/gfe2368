/*
 *imu.h
 */

#ifndef IMU_H_
#define IMU_H_

#include "lpc23xx-types.h"

void imu_isr() __attribute__ ((interrupt("IRQ")));

/*GPIO 0 wiring*/
//P0.00 CAN
//P0.01 CAN
//P0.02 UART
//P0.03 UART
//P0.04 CAN
//P0.05 ----
//P0.06 ---- = ACCEL SA0
//P0.07 ---- = ACCEL INT2
//P0.08 ---- = ACCEL INT1
//P0.09 ---- = ACCEL CS
//P0.10 SDA2 = MAG SDA
//P0.11 SCL2 = MAG SCL
//-----
//P0.15 ---- = MAG INT1
//P0.16 ----
//P0.17 ----
//P0.18 ---- = MAG INT2
//P0.19 SDA1 = ACCEL SDA
//P0.20 SCL1 = ACCEL SCL
//P0.21 ---- = MAG SA
//P0.22 ---- = MAG DRDY
//P0.23 ---- = GYRO INT1
//P0.24 ---- = GYRO INT2
//P0.25 ---- = GYRO CS
//P0.26 ---- = GYRO SA0
//P0.27 SDA0 = GYRO SDA
//P0.28 SCL0 = GYRO SCL
//P0.29 USB
//P0.30 USB

#define ACCEL_SA0	(1<<6)
#define ACCEL_INT2	(1<<7)
#define ACCEL_INT1	(1<<8)
#define ACCEL_CS	(1<<9)
#define MAG_INT1	(1<<15)
#define MAG_INT2	(1<<18)
#define MAG_SA		(1<<21)
#define MAG_DRDY	(1<<22)
#define GYRO_INT1	(1<<23)
#define GYRO_INT2	(1<<24)
#define GYRO_CS		(1<<25)
#define	GYRO_SA0	(1<<26)
/*
 * If the sensor interrupt is enabled and not active, but its drdy line is active
 * and there is no i2c call going to clear it, it is stuck.
 */
#define L3G4200D_STUCK ((IO0IntEnR & GYRO_INT2) && \
        !(IO0IntStatR & GYRO_INT2) && (FIO0PIN & GYRO_INT2) && \
        !i2c_active(I2C0))

#define LIS331HH_STUCK ((IO0IntEnR & ACCEL_INT1) && \
        !(IO0IntStatR & ACCEL_INT1) && (FIO0PIN & ACCEL_INT1) && \
        !i2c_active(I2C1))
/*
the LSM303DLH_M DRDY line is high always except when a new sample is being
loaded into the data registers
#define LSM303DLH_M_STUCK ((IO0IntEnR & MAG_DRDY) && \
        !(IO0IntStatR & MAG_DRDY) && (FIO0PIN & MAG_DRDY) && \
        !is_binsem_locked(&i2c2_binsem_g))
*/

#define ISOC0_IN_EP 0x83
#define ISOC1_IN_EP 0x86
#define ISOC2_IN_EP 0x89
#define ISOC3_IN_EP 0x8C

#define BULK0_IN_EP 0x82
#define BULK1_IN_EP 0x85
#define BULK2_IN_EP 0x88
#define BULK3_IN_EP 0x8B
#define ACC_EP BULK0_IN_EP
#define GYR_EP BULK1_IN_EP
#define MAG_EP BULK2_IN_EP
#define CAC_EP BULK3_IN_EP

#define CTRL_OUT_EP             0x00
#define ADDR_ACC                0x80
#define ADDR_GYR                0x40
#define ADDR_MAG                0x20
#define ADDR_CAC                0x10
#define ADDR_ALL                0xF0
#define INST_RESET              0x01
#define INST_GO                 0x02
#define INST_STOP               0x03
#define INST_SET_SPEED          0x04

#define IMU_INST(X)             ((X) & 0x0F)
#define IMU_ADDR(X)             ((X) & 0xF0)

#define IMU_PACKET_LENGTH 13

#define MAX_PACKET_SIZE 64

#define LE_WORD(x)              ((x)&0xFF),((x)>>8)


static const uint8_t imu_descriptor[] = {

    // device descriptor
    0x12,
    DESC_DEVICE,
    LE_WORD(0x0200),                    // bcdUSB
    0xFF,                               // bDeviceClass
    0xFF,                               // bDeviceSubClass
    0xFF,                               // bDeviceProtocol
    MAX_PACKET_SIZE0,                   // bMaxPacketSize
    LE_WORD(0xFFFF),                    // idVendor
    LE_WORD(0x0005),                    // idProduct
    LE_WORD(0x0100),                    // bcdDevice
    0x01,                               // iManufacturer
    0x02,                               // iProduct
    0x03,                               // iSerialNumber
    0x01,                               // bNumConfigurations

    // configuration descriptor
    0x09,
    DESC_CONFIGURATION,
    LE_WORD(46),                        // wTotalLength
    0x01,                               // bNumInterfaces
    0x01,                               // bConfigurationValue
    0x00,                               // iConfiguration
    0x80,                               // bmAttributes
    0xF0,                               // bMaxPower

    // data class interface descriptor
    0x09,
    DESC_INTERFACE,
    0x00,                                // bInterfaceNumber
    0x00,                                // bAlternateSetting
    0x04,                                // bNumEndPoints
    0xFF,                                // bInterfaceClass = vendor
    0xFF,                                // bInterfaceSubClass
    0xFF,                                // bInterfaceProtocol
    0x00,                                // iInterface
    // Accelerometer isoc in ep
    0x07,
    DESC_ENDPOINT,
    ACC_EP,                              // bEndpointAddress
    0x02,                                // bmAttributes = bulk
    LE_WORD(IMU_PACKET_LENGTH),          // wMaxPacketSize
    0x01,                                // bInterval
    // Gyroscope isoc in ep
    0x07,
    DESC_ENDPOINT,
    GYR_EP,                              // bEndpointAddress
    0x02,                                // bmAttributes = bulk
    LE_WORD(IMU_PACKET_LENGTH),          // wMaxPacketSize
    0x01,                                // bInterval
    // Magnetometer isoc in ep
    0x07,
    DESC_ENDPOINT,
    MAG_EP,                              // bEndpointAddress
    0x02,                                // bmAttributes = bulk
    LE_WORD(IMU_PACKET_LENGTH),          // wMaxPacketSize
    0x01,                                // bInterval
    // Compass Accelerometer isoc in ep
    0x07,
    DESC_ENDPOINT,
    CAC_EP,                              // bEndpointAddress
    0x02,                                // bmAttributes = bulk
    LE_WORD(IMU_PACKET_LENGTH),          // wMaxPacketSize
    0x01,                                // bInterval

    // string descriptors
    0x04,                                //bLength
    DESC_STRING,                         //bDescriptorType
    LE_WORD(0x0409),                     //wLANGID[0]

    0x0A,
    DESC_STRING,
    'P', 0, 'S', 0, 'A', 0, 'S', 0,

    0x12,
    DESC_STRING,
    'T', 0, 'h', 0, 'e', 0, 'o', 0, '-', 0, 'I', 0, 'M', 0, 'U', 0,

    0x1A,
    DESC_STRING,
    'A', 0, 'V', 0, '3', 0, ' ', 0, 'O', 0, 'c', 0, 't', 0, ' ', 0, '2', 0,
    '0', 0, '1', 0, '2', 0,

    // terminating zero
    0
};

#endif /* IMU_H_ */
