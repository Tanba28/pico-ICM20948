#include "ICM20948.hpp"

#include "stdio.h"
#include "pico/stdlib.h"

ICM20948::ICM20948(i2c_inst_t *_i2c,uint sda_pin,uint scl_pin ,uint baudrate):
i2c(_i2c){
    i2c_init(i2c, baudrate);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    pwrMgmt1Configuration();
    pwrMgmt2Configuration();
    intPinCfgConfiguration();
    gyroConfig1Configuration();
    gyroSamplingRateDiv();
    accelConfig1Configuration();
    accelSamplingRateDiv();
}

bool ICM20948::isWhoAmI(){
    uint8_t var;
    uint8_t buf;

    bankChange(USER_BANK::BANK0);   

    var = BANK0_REG::WHO_AM_I;
    i2c_write_blocking(i2c,ICM_ADDRESS,&var,1,true);
    i2c_read_blocking(i2c,ICM_ADDRESS,&buf,1,false);

    printf("%x\n",buf);

    if(buf == 0xEA){
        return true;
    }
    return false;
}

void ICM20948::measurement(){
    uint8_t var;
    uint8_t buf[14];

    bankChange(USER_BANK::BANK0);

    var = BANK0_REG::ACCEL_XOUT_H;
    i2c_write_blocking(i2c,ICM_ADDRESS,&var,1,true);
    i2c_read_blocking(i2c,ICM_ADDRESS,buf,14,false);

    accel_raw[0] = (uint16_t)buf[0] << 8 | buf[1];
    accel_raw[1] = (uint16_t)buf[2] << 8 | buf[3];
    accel_raw[2] = (uint16_t)buf[4] << 8 | buf[5];
    gyro_raw[0] = (uint16_t)buf[6] << 8 | buf[7];
    gyro_raw[1] = (uint16_t)buf[8] << 8 | buf[9];
    gyro_raw[2] = (uint16_t)buf[10] << 8 | buf[11];

    accel.data[0] = ((int16_t)accel_raw[0] * accel_res) / 32768.0;
    accel.data[1] = ((int16_t)accel_raw[1] * accel_res) / 32768.0;
    accel.data[2] = ((int16_t)accel_raw[2] * accel_res) / 32768.0;
    gyro.data[0] = ((int16_t)gyro_raw[0] * gyro_res) / 32768.0;
    gyro.data[1] = ((int16_t)gyro_raw[1] * gyro_res) / 32768.0;
    gyro.data[2] = ((int16_t)gyro_raw[2] * gyro_res) / 32768.0;
}
Vector ICM20948::getGyro(){
    return gyro;
}
Vector ICM20948::getAceel(){
    return accel;
}

void ICM20948::bankChange(USER_BANK user_bank){
    uint8_t var[2];

    this->reg_bank_sel.contents.user_bank = user_bank;
    this->reg_bank_sel.contents.reserve1 = 0;
    this->reg_bank_sel.contents.reserve2 = 0;

    var[0] = BANK0_REG::REG_BANK_SEL_0;
    var[1] = this->reg_bank_sel.byte;
    i2c_write_blocking(i2c,ICM_ADDRESS,var,2,false);
}

void ICM20948::pwrMgmt1Configuration(CLKSEL clksel,TEMP_DIS temp_dis,LP_EN lp_en,SLEEP_IN sleep,DEVICE_RESET device_reset){
    uint8_t var[2];

    bankChange(USER_BANK::BANK0);

    this->pwr_mgmt_1.contents.clksel = clksel;
    this->pwr_mgmt_1.contents.temp_dis = temp_dis;
    this->pwr_mgmt_1.contents.lp_en = lp_en;
    this->pwr_mgmt_1.contents.sleep = sleep;
    this->pwr_mgmt_1.contents.device_reset = device_reset;
    this->pwr_mgmt_1.contents.reserve = 0;
    
    var[0] = BANK0_REG::PWR_MGMT_1;
    var[1] = this->pwr_mgmt_1.byte;
    i2c_write_blocking(i2c,ICM_ADDRESS,var,2,false);
}
void ICM20948::pwrMgmt2Configuration(DISABLE_GYRO disable_gyro,DISABLE_ACCEL disable_accel){
    uint8_t var[2];

    bankChange(USER_BANK::BANK0);

    this->pwr_mgmt_2.contents.disable_gyro = disable_gyro;
    this->pwr_mgmt_2.contents.disable_accel = disable_accel;
    this->pwr_mgmt_2.contents.reserve = 0;

    var[0] = BANK0_REG::PWR_MGMT_2;
    var[1] = this->pwr_mgmt_2.byte;
    i2c_write_blocking(i2c,ICM_ADDRESS,var,2,false);
}

void ICM20948::intPinCfgConfiguration(BYPASS_EN bypass_em,FSYNC_INT_MODE_EN fsync_int_mode_en,ACTL_FSYNC actl_fsyc,INT_ANYRD_2CLEAR int_anyrd_2clear,INT1_LATCH_EN int1_latch_en,INT1_OPEN int1_open,INT1_ACTL int1_actl){
    uint8_t var[2];

    bankChange(USER_BANK::BANK0);

    this->int_pin_cfg.contents.bypass_em = bypass_em;
    this->int_pin_cfg.contents.fsync_int_mode_en = fsync_int_mode_en;
    this->int_pin_cfg.contents.actl_fsyc = actl_fsyc;
    this->int_pin_cfg.contents.int_anyrd_2clear = int_anyrd_2clear;
    this->int_pin_cfg.contents.int1_latch_en = int1_latch_en;
    this->int_pin_cfg.contents.int1_open = int1_open;
    this->int_pin_cfg.contents.int1_actl = int1_actl;
    this->int_pin_cfg.contents.reserve = 0;

    var[0] = BANK0_REG::INT_PIN_CFG;
    var[1] = this->int_pin_cfg.byte;
    i2c_write_blocking(i2c,ICM_ADDRESS,var,2,false);
}
void ICM20948::gyroSamplingRateDiv(uint8_t div){
    uint8_t var[2];

    bankChange(USER_BANK::BANK2);

    var[0] = BANK2_REG::GYRO_SMPLRT_DIV;
    var[1] = div;
    i2c_write_blocking(i2c,ICM_ADDRESS,var,2,false);
}
void ICM20948::gyroConfig1Configuration(GYRO_FCHOICE gyro_fchoice,GYRO_FS_SEL gyro_fs_sel,GYRO_DLPFCFG gyro_dlpgcfg){
    uint8_t var[2];

    bankChange(USER_BANK::BANK2);

    this->gyro_config_1.contents.gyro_fchoice = gyro_fchoice;
    this->gyro_config_1.contents.gyro_fs_sel = gyro_fs_sel;
    this->gyro_config_1.contents.gyro_dlpgcfg = gyro_dlpgcfg;
    this->gyro_config_1.contents.reserve = 0;

    var[0] = BANK2_REG::GYRO_CONFIG_1;
    var[1] = this->gyro_config_1.byte;
    i2c_write_blocking(i2c,ICM_ADDRESS,var,2,false);

    setGyroRes();
}
void ICM20948::accelSamplingRateDiv(uint8_t div){
    uint8_t var[2];

    bankChange(USER_BANK::BANK2);

    var[0] = BANK2_REG::ACCEL_SMPLRT_DIV_2;
    var[1] = div;
    i2c_write_blocking(i2c,ICM_ADDRESS,var,2,false);
}
void ICM20948::accelConfig1Configuration(ACCEL_FCHOICE aceel_fchoice,ACCEL_FS_SEL aceel_fs_sel,ACCEL_DLPFCFG aceel_dlpgcfg){
    uint8_t var[2];

    bankChange(USER_BANK::BANK2);

    this->accel_config.contents.aceel_fchoice = aceel_fchoice;
    this->accel_config.contents.aceel_fs_sel = aceel_fs_sel;
    this->accel_config.contents.aceel_dlpgcfg = aceel_dlpgcfg;
    this->accel_config.contents.reserve = 0;

    var[0] = BANK2_REG::ACCEL_CONFIG;
    var[1] = this->accel_config.byte;
    i2c_write_blocking(i2c,ICM_ADDRESS,var,2,false);

    setAccelRes();
}

void ICM20948::setAccelRes(){
    switch (this->accel_config.contents.aceel_fs_sel)
    {
    case ACCEL_FS_SEL::_2G:
        this->accel_res = 2;
        break;
    case ACCEL_FS_SEL::_4G:
        this->accel_res = 4;
        break;
    case ACCEL_FS_SEL::_8G:
        this->accel_res = 8;
        break;
    case ACCEL_FS_SEL::_16G:
        this->accel_res = 16;
        break;
    }
}
void ICM20948::setGyroRes(){
    switch (this->gyro_config_1.contents.gyro_fs_sel)
    {
    case GYRO_FS_SEL::DPS_250:
        this->gyro_res = 250;
        break;
    case GYRO_FS_SEL::DPS_500:
        this->gyro_res = 500;
        break;
    case GYRO_FS_SEL::DPS_1000:
        this->gyro_res = 1000;
        break;
    case GYRO_FS_SEL::DPS_2000:
        this->gyro_res = 2000;
        break;
    }
}