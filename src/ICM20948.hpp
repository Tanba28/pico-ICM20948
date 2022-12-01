#ifndef __PICO_ICM20948__
#define __PICO_ICM20948__

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "pico_i2c.hpp"

class ICM20948{
    public:
        /* REGISTER MAP */
        // PWR_MGMT_1
        enum class DEVICE_RESET : uint8_t{
            NO_RESET = 0x00,
            RESET = 0x01
        };
        enum class SLEEP_IN : uint8_t{
            NO_SLEEP = 0x00,
            SLEEP = 0x01                
        };
        enum class LP_EN : uint8_t{ // Low Power Mode
            DISABLE = 0x00,
            ENABLE = 0x01
        };
        enum class TEMP_DIS : uint8_t{ // Temperature Enable or Disable
            DISABLE = 0x01,
            ENABLE = 0x00
        };
        enum class CLKSEL : uint8_t{
            INTERNAL = 0x00,
            AUTO_SELECT = 0x01,
            RESET = 0x07
        };
        // PWR_MGMT_2
        enum class DISABLE_ACCEL : uint8_t{
            DISABLE = 0x07,
            ENABLE = 0x00
        };
        enum class DISABLE_GYRO : uint8_t{
            DISABLE = 0x07,
            ENABLE = 0x00
        };
        // INT_PIN_CFG
        enum class INT1_ACTL : uint8_t{ //Logic Level
            ACTIVE_LOW = 0x01,
            ACTIVE_HIGH = 0x00
        };
        enum class INT1_OPEN : uint8_t{
            OPEN_DRAIN = 0x01,
            OPEN_PUSH_PULL = 0x00
        };
        enum class INT1_LATCH_EN : uint8_t{
            STEP = 0x01,
            PULSE = 0x00
        };
        enum class INT_ANYRD_2CLEAR : uint8_t{ //When auto, any register is read and cleared. In manual mode, INT_STATUS must be read.
            AUTO = 0x01,
            MANUAL = 0x00
        };
        enum class ACTL_FSYNC : uint8_t{
            ACTIVE_LOW = 0x01,
            ACTIVE_HIGH = 0x00
        };
        enum class FSYNC_INT_MODE_EN : uint8_t{
            DISABLE = 0x00,
            ENABLE = 0x01
        };
        enum class BYPASS_EN : uint8_t{
            DISABLE = 0x00,
            ENABLE = 0x01
        };
        
        // TODO:INTERRUPT
        // TODO:FSYNC
        // TODO:FIFO

        // GYRO_CONFIG_1
        enum class GYRO_DLPFCFG{ //3db BANDWIDH
            BAND_196HZ = 0x00,
            BAND_152HZ = 0x01,
            BAND_120HZ = 0x02,
            BAND_51HZ = 0x03,
            BAND_24HZ = 0x04,
            BAND_12HZ = 0x05,
            BAND_6HZ = 0x06,
            BAND_361HZ = 0x07
        };
        enum class GYRO_FS_SEL : uint8_t{
            DPS_250 = 0x00,
            DPS_500 = 0x01,
            DPS_1000 = 0x02,
            DPS_2000 = 0x03
        };
        enum class GYRO_FCHOICE{
            BYPASS = 0x00,
            ENABLE = 0x01
        };
        
        // TODO:GYRO CONFIG

        // ACCEL_CONFIG
        enum class ACCEL_DLPFCFG{ //3db BANDWIDH
            BAND_246HZ = 0x00,
            BAND_111HZ = 0x02,
            BAND_50HZ = 0x03,
            BAND_24HZ = 0x04,
            BAND_12HZ = 0x05,
            BAND_6HZ = 0x06,
            BAND_473HZ = 0x07
        };
        enum class ACCEL_FS_SEL : uint8_t{
            _2G = 0x00,
            _4G = 0x01,
            _8G = 0x02,
            _16G = 0x03
        };
        enum class ACCEL_FCHOICE{
            BYPASS = 0x00,
            ENABLE = 0x01
        };

        enum class USER_BANK{
            BANK0 = 0x00,
            BANK1 = 0x01,
            BANK2 = 0x02,
            BANK3 = 0x03
        };

        ICM20948(i2c_inst_t *_i2c,uint sda_pin,uint scl_pin ,uint baudrate);
        ICM20948(PicoI2C *_i2c);
        bool isWhoAmI();

        void measurement();
        void getGyro(float *buf);
        void getAceel(float *buf);

        void bankChange(USER_BANK user_bank);

        void pwrMgmt1Configuration(
            CLKSEL clksel = CLKSEL::AUTO_SELECT,
            TEMP_DIS temp_dis = TEMP_DIS::ENABLE,
            LP_EN lp_en = LP_EN::DISABLE,
            SLEEP_IN sleep = SLEEP_IN::NO_SLEEP,
            DEVICE_RESET device_reset = DEVICE_RESET::NO_RESET);
        void pwrMgmt2Configuration(
            DISABLE_GYRO disable_gyro = DISABLE_GYRO::ENABLE,
            DISABLE_ACCEL disable_accel = DISABLE_ACCEL::ENABLE);
        void intPinCfgConfiguration(
            BYPASS_EN bypass_em = BYPASS_EN::DISABLE,
            FSYNC_INT_MODE_EN fsync_int_mode_en = FSYNC_INT_MODE_EN::DISABLE,
            ACTL_FSYNC actl_fsyc = ACTL_FSYNC::ACTIVE_HIGH,
            INT_ANYRD_2CLEAR int_anyrd_2clear = INT_ANYRD_2CLEAR::MANUAL,
            INT1_LATCH_EN int1_latch_en = INT1_LATCH_EN::PULSE,
            INT1_OPEN int1_open = INT1_OPEN::OPEN_PUSH_PULL,
            INT1_ACTL int1_actl = INT1_ACTL::ACTIVE_HIGH);
        void gyroSamplingRateDiv(uint8_t div = 0x00);
        void gyroConfig1Configuration(
            GYRO_FCHOICE gyro_fchoice = GYRO_FCHOICE::ENABLE,
            GYRO_FS_SEL gyro_fs_sel = GYRO_FS_SEL::DPS_250,
            GYRO_DLPFCFG gyro_dlpgcfg = GYRO_DLPFCFG::BAND_51HZ);
        void accelSamplingRateDiv(uint8_t div = 0x00);
        void accelConfig1Configuration(
            ACCEL_FCHOICE aceel_fchoice = ACCEL_FCHOICE::ENABLE,
            ACCEL_FS_SEL aceel_fs_sel = ACCEL_FS_SEL::_2G,
            ACCEL_DLPFCFG aceel_dlpgcfg = ACCEL_DLPFCFG::BAND_24HZ);

    private:

        float gyro[3];
        uint16_t gyro_raw[3];
        float accel[3];
        uint16_t accel_raw[3];

        uint8_t accel_res;
        uint16_t gyro_res;

        PicoI2C *i2c = NULL;

        uint8_t ICM_ADDRESS  = 0x68;
        
        void setAccelRes();
        void setGyroRes();

        void i2c_write(uint8_t reg,const uint8_t *src,size_t len);
        void i2c_read(uint8_t reg,uint8_t *dst,size_t len);

        enum BANK0_REG : uint8_t{
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
            REG_BANK_SEL_0 = 0x7F
        };

        enum BANK1_REG : uint8_t{
            SELF_TEST_X_GYRO = 0x02,
            SELF_TEST_Y_GYRO = 0x03,
            SELF_TEST_z_GYRO = 0x04,
            SELF_TEST_X_ACCEL = 0x0E,
            SELF_TEST_Y_ACCEL = 0x0F,
            SELF_TEST_z_ACCEL = 0x10,
            XA_OFFS_H = 0x14,
            XA_OFFS_L = 0x15,
            YA_OFFS_H = 0x17,
            YA_OFFS_L = 0x18,
            ZA_OFFS_H = 0x1A,
            ZA_OFFS_L = 0x1B,
            TIMEBASE_CORRECTION_PLL = 0x28,
            REG_BANK_SEL_1 = 0x7F
        };

        enum BANK2_REG : uint8_t{
            GYRO_SMPLRT_DIV = 0x00,
            GYRO_CONFIG_1 = 0x01,
            GYRO_CONFIG_2 = 0x02,
            XG_OFFS_USRH = 0x03,
            XG_OFFS_USRL = 0x04,
            YG_OFFS_USRH = 0x05,
            YG_OFFS_USRL = 0x06,
            ZG_OFFS_USRH = 0x07,
            ZG_OFFS_USRL = 0x08,
            ODR_ALIGN_EN = 0x09,
            ACCEL_SMPLRT_DIV_1 = 0x10,
            ACCEL_SMPLRT_DIV_2 = 0x11,
            ACCEL_INTEL_CTRL = 0x12,
            ACCEL_WOM_THR = 0x13,
            ACCEL_CONFIG = 0x14,
            ACCEL_CONFIG_2 = 0x15,
            FSYNC_CONFIG = 0x52,
            TEMP_CONFIG = 0x53,
            MOD_CTRL_USR = 0x54,
            REG_BANK_SEL_2 = 0x7F
        };

        enum BANK3_REG : uint8_t{
            I2C_MST_ODR_CONFIG = 0x00,
            I2C_MST_CTRL = 0x01,
            I2C_MST_DELAY_CTRL = 0x02,
            I2C_SLV0_ADDR = 0x03,
            I2C_SLV0_REG = 0x04,
            I2C_SLV0_CTRL = 0x05,
            I2C_SLV0_DO = 0x06,
            I2C_SLV1_ADDR = 0x07,
            I2C_SLV1_REG = 0x08,
            I2C_SLV1_CTRL = 0x09,
            I2C_SLV1_DO = 0x0A,
            I2C_SLV2_ADDR = 0x0B,
            I2C_SLV2_REG = 0x0C,
            I2C_SLV2_CTRL = 0x0D,
            I2C_SLV2_DO = 0x0E,
            I2C_SLV3_ADDR = 0x0F,
            I2C_SLV3_REG = 0x10,
            I2C_SLV3_CTRL = 0x11,
            I2C_SLV3_DO = 0x12,
            I2C_SLV4_ADDR = 0x13,
            I2C_SLV4_REG = 0x14,
            I2C_SLV4_CTRL = 0x15,
            I2C_SLV4_DO = 0x16,
            I2C_SLV4_DI = 0x17,
            REG_BANK_SEL_3 = 0x7F
        };

        #pragma pack(push, 1)

        union PWR_MGMT_1{
            struct {
                CLKSEL clksel : 3;
                TEMP_DIS temp_dis : 1;
                bool reserve : 1;
                LP_EN lp_en : 1;
                SLEEP_IN sleep :1;
                DEVICE_RESET device_reset :1;
            } contents;
            uint8_t byte;
        } pwr_mgmt_1;

        
        union PWR_MGMT_2{
            struct {
                DISABLE_GYRO disable_gyro :3;
                DISABLE_ACCEL disable_accel :3;
                uint8_t reserve :2;
            } contents;
            uint8_t byte;
        } pwr_mgmt_2;

        union INT_PIN_CFG{
            struct {
                bool reserve :1;
                BYPASS_EN bypass_em :1;
                FSYNC_INT_MODE_EN fsync_int_mode_en :1;
                ACTL_FSYNC actl_fsyc :1;
                INT_ANYRD_2CLEAR int_anyrd_2clear :1;
                INT1_LATCH_EN int1_latch_en :1;
                INT1_OPEN int1_open :1;
                INT1_ACTL int1_actl :1;
            } contents;
            uint8_t byte;
        } int_pin_cfg;

        union GYRO_CONFIG_1{
            struct {
                GYRO_FCHOICE gyro_fchoice :1;
                GYRO_FS_SEL gyro_fs_sel :2;
                GYRO_DLPFCFG gyro_dlpgcfg :3;
                bool reserve :1;
            } contents;
            uint8_t byte;
        } gyro_config_1;

        union ACCEL_CONFIG{
            struct {
                ACCEL_FCHOICE aceel_fchoice :1;
                ACCEL_FS_SEL aceel_fs_sel :2;
                ACCEL_DLPFCFG aceel_dlpgcfg :3;
                bool reserve :1;
            } contents;
            uint8_t byte;
        } accel_config;     

        union REG_BANK_SEL{
            struct{
                uint8_t reserve1 :4;
                USER_BANK user_bank :2;
                uint8_t reserve2 :2;
            } contents;
            uint8_t byte;
        } reg_bank_sel;
        #pragma pack(pop)
};

#endif