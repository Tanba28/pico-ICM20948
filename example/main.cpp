#include "stdio.h"
#include "pico/stdlib.h"

#include "ICM20948.hpp"
#include "pico_i2c.hpp"
#include "task_base.hpp"

const uint LED_R_PIN = 18;
const uint LED_G_PIN = 19;
const uint LED_B_PIN = 20;

class ICM20948Task : public TaskBase{
    public:
        ICM20948Task():TaskBase("icm20948_task",3,4096){
        }
        void task(){
            gpio_init(18);
            gpio_set_dir(18, GPIO_OUT);

            float gyro[3];
            float accel[3];
            PicoDmaI2C::createInstance(i2c1,2,3,100*1000);
            i2c = PicoDmaI2C::getInstance();
            ICM20948 icm = ICM20948(i2c);

            for(;;){
                icm.measurement();
                icm.getGyro(gyro);
                icm.getAceel(accel);

                printf("%+f,%+f,%+f,%+f,%+f,%+f\n",accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]);

                gpio_put(LED_G_PIN,!gpio_get(LED_G_PIN));
                gpio_put(LED_R_PIN,!gpio_get(LED_R_PIN));

                vTaskDelay(10);
            }
        }
    private:
        PicoDmaI2C *i2c;
};

int main() {
    // Vector gyro;
    // Vector accel;

    stdio_init_all();

    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_init(LED_G_PIN);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);
    gpio_put(LED_R_PIN,0);
    gpio_put(LED_G_PIN,1);
    gpio_put(LED_B_PIN,1);

    sleep_ms(1000);

    ICM20948Task task = ICM20948Task();
    task.createTask();

    vTaskStartScheduler();

    // ICM20948 icm = ICM20948(i2c1,2,3,100*1000);
    while (true) {
        // icm.isWhoAmI();
        // icm.measurement();
        // gyro = icm.getGyro();
        // accel = icm.getAceel();

        // printf("%+f,%+f,%+f,%+f,%+f,%+f\n",accel.data[0],accel.data[1],accel.data[2],gyro.data[0],gyro.data[1],gyro.data[2]);

        // gpio_put(LED_G_PIN,!gpio_get(LED_G_PIN));
        // gpio_put(LED_R_PIN,!gpio_get(LED_R_PIN));

        // sleep_ms(1);
    }
}