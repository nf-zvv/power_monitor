#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "INA226.h"


int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ina226_reset();
    ina226_setAverage(INA226_4_SAMPLES);
    ina226_setBusVoltageConversionTime(INA226_332_us);
    ina226_setShuntVoltageConversionTime(INA226_1100_us);

    float shunt = 0.00206f;
    float current_LSB_mA = 2.0f;
    float current_zero_offset_mA = 0.0f;
    uint16_t bus_V_scaling_e4 = 10000;

    ina226_configure(shunt, current_LSB_mA, current_zero_offset_mA, bus_V_scaling_e4);

    float voltage, current, power, shunt_volt;

    while (true) {
        voltage = ina226_getBusVoltage();
        shunt_volt = ina226_getShuntVoltage();
        current = ina226_getCurrent();
        power = ina226_getPower();
        printf("%.3f;%.3f;%.3f;%.6f\n", voltage, current, power, shunt_volt);
        sleep_ms(1000);
    }
}
