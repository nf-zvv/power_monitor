#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"

#include "INA226.h"

float battery_charge(float voltage)
{
    if (voltage >= 4.25) return 100.0;
    if (voltage >= 3.75 && voltage < 4.25) return 42*voltage-78.5;
    if (voltage >= 3.50 && voltage < 3.75) return 236*voltage-806;
    if (voltage >= 2.99 && voltage < 3.50) return 40*voltage-120;
    if (voltage < 2.99) return 0.0;
}

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

    float shunt = 0.002292f;
    float current_LSB_mA = 1.0f;
    float current_zero_offset_mA = 0.0f;
    uint16_t bus_V_scaling_e4 = 10000;

    ina226_configure(shunt, current_LSB_mA, current_zero_offset_mA, bus_V_scaling_e4);

    char datetime_buf[256];
    char *datetime_str = &datetime_buf[0];

    // Start on Monday 21th of July 2025 09:00:00
    datetime_t t = {
        .year = 2025,
        .month = 07,
        .day = 21,
        .dotw = 1, // 0 is Sunday, so 5 is Friday
        .hour = 9,
        .min = 00,
        .sec = 00
    };

    // Start the RTC
    rtc_init();
    rtc_set_datetime(&t);
    sleep_us(64);

    float voltage, current, power, shunt_volt, charge;

    while (true) {
        rtc_get_datetime(&t);
        datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
        printf("%02d.%02d.%d;%02d:%02d:%02d;", t.day, t.month, t.year, t.hour, t.min, t.sec);

        voltage = ina226_getBusVoltage();
        shunt_volt = ina226_getShuntVoltage();
        current = ina226_getCurrent();
        power = ina226_getPower();
        charge = battery_charge(voltage);
        printf("%.3f;%.3f;%.3f;%.6f;%.1f%%\n", datetime_str, voltage, current, power, shunt_volt, charge);
        sleep_ms(1000);
    }
}
