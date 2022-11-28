#ifndef __PICO_ICM20948__
#define __PICO_ICM20948__

#include "pico/stdlib.h"
#include "hardware/i2c.h"

class ICM20948{
    public:
        /* TODO:Error handle */
        enum class RESULT{
            OK = 0x01,
            FAIL = 0x02,
        };

        ICM20948(i2c_inst_t *_i2c,uint sda_pin,uint scl_pin ,uint baudrate);
        bool isWhoAmI();

    private:
        i2c_inst_t *i2c;

        enum REG : uint8_t{
            ADDRESS  = 0x68,
            WHO_AM_I = 0x00,
            USER_CTRL = 0x03,
            LP_CONFIG = 0x05,
            PWR_MGMT_1 = 0x06,
            PWR_MGMT_2 = 0x07,
            INT_PIN_CFG = 0x0F,
            INT_ENABLE = 0x10,
            INT_ENABLE_1 = 0x11,
            INT_ENABLE_2 = 0x12,
            INT_ENABLE_3 = 0x13,
            I2C_MST_STATUS = 0x17,
            INT_STATUS = 0x19,
            INT_STATUS_1 = 0x1A,
            INT_STATUS_2 = 0x1B,
            INT_STATUS_3 = 0x1C,
            DELAY_TIMEH = 0x28,
            DELAY_TIMEL = 0x29,
            ACCEL_XOUT_H = 0x2D,
            ACCEL_XOUT_L = 0x2E,
            ACCEL_YOUT_H = 0x2F,
            ACCEL_YOUT_L = 0x30,
            ACCEL_ZOUT_H = 0x31,
            ACCEL_ZOUT_L = 0x32,
            GYRO_XOUT_H = 0x33,
            GYRO_XOUT_L = 0x34,
            GYRO_YOUT_H = 0x35,
            GYRO_YOUT_L = 0x36,
            GYRO_ZOUT_H = 0x37,
            GYRO_ZOUT_L = 0x38,
            TEMP_OUT_H = 0x39,
            TEMP_OUT_L = 0x3A,
            EXT_SLV_SENS_DATA_00 = 0x3B,
            EXT_SLV_SENS_DATA_01 = 0x3C,
            EXT_SLV_SENS_DATA_02 = 0x3D,
            EXT_SLV_SENS_DATA_03 = 0x3E,
            EXT_SLV_SENS_DATA_04 = 0x3F,
            EXT_SLV_SENS_DATA_05 = 0x40,
            EXT_SLV_SENS_DATA_06 = 0x41,
            EXT_SLV_SENS_DATA_07 = 0x42,
            EXT_SLV_SENS_DATA_08 = 0x43,
            EXT_SLV_SENS_DATA_09 = 0x44,
            EXT_SLV_SENS_DATA_10 = 0x45,
            EXT_SLV_SENS_DATA_11 = 0x46,
            EXT_SLV_SENS_DATA_12 = 0x47,
            EXT_SLV_SENS_DATA_13 = 0x48,
            EXT_SLV_SENS_DATA_14 = 0x49,
            EXT_SLV_SENS_DATA_15 = 0x4A,
            EXT_SLV_SENS_DATA_16 = 0x4B,
            EXT_SLV_SENS_DATA_17 = 0x4C,
            EXT_SLV_SENS_DATA_18 = 0x4D,
            EXT_SLV_SENS_DATA_19 = 0x4E,
            EXT_SLV_SENS_DATA_20 = 0x4F,
            EXT_SLV_SENS_DATA_21 = 0x50,
            EXT_SLV_SENS_DATA_22 = 0x51,
            EXT_SLV_SENS_DATA_23 = 0x52,
            FIFO_EN_1 = 0x66,
            FIFO_EN_2 = 0x67,
            FIFO_RST = 0x68,
            FIFO_MODE = 0x69,
            FIFO_COUNTH = 0x70,
            FIFO_COUNTL = 0x71,
            FIFO_R_W = 0x72,
            DATA_RDY_STATUS = 0x74,
            FIFO_CFG = 0x76,
            REG_BANK_SEL = 0x7F
        };
};

#endif