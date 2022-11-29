#include "stdio.h"
#include "pico/stdlib.h"

#include "ICM20948.hpp"

int main() {
    Vector gyro;
    Vector accel;

    stdio_init_all();

    const uint LED_R_PIN = 18;
    const uint LED_G_PIN = 19;
    const uint LED_B_PIN = 20;
    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_init(LED_G_PIN);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);
    gpio_put(LED_R_PIN,1);
    gpio_put(LED_G_PIN,1);
    gpio_put(LED_B_PIN,1);

    sleep_ms(1000);

    ICM20948 icm = ICM20948(i2c1,2,3,100*1000);
    while (true) {
        // icm.isWhoAmI();
        icm.measurement();
        gyro = icm.getGyro();
        accel = icm.getAceel();

        printf("%+f,%+f,%+f,%+f,%+f,%+f\n",accel.data[0],accel.data[1],accel.data[2],gyro.data[0],gyro.data[1],gyro.data[2]);

        gpio_put(LED_G_PIN,!gpio_get(LED_G_PIN));

        // sleep_ms(1);
    }
}