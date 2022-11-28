#include "ICM20948.hpp"

#include "stdio.h"
#include "pico/stdlib.h"

ICM20948::ICM20948(i2c_inst_t *_i2c,uint sda_pin,uint scl_pin ,uint baudrate):
i2c(_i2c){
    i2c_init(i2c, baudrate);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
}

bool ICM20948::isWhoAmI(){
    uint8_t var;
    uint8_t buf;

    var = REG::WHO_AM_I;
    i2c_write_blocking(i2c,REG::ADDRESS,&var,1,true);
    i2c_read_blocking(i2c,REG::ADDRESS,&buf,1,false);

    printf("%x\n",buf);

    if(buf == 0xEA){
        return true;
    }
    return false;
}