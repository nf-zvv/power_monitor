#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"

#include "INA226.h"

#define BUFFER_LENGTH 25
volatile uint8_t buffer[BUFFER_LENGTH];
volatile uint8_t buffer_index;

float battery_charge(float voltage)
{
    if (voltage >= 4.25) return 100.0;
    if (voltage >= 3.75 && voltage < 4.25) return 42*voltage-78.5;
    if (voltage >= 3.50 && voltage < 3.75) return 236*voltage-806;
    if (voltage >= 2.99 && voltage < 3.50) return 40*voltage-120;
    if (voltage < 2.99) return 0.0;
}

bool repeating_timer_callback(__unused struct repeating_timer *t)
{
    float voltage, current, power, shunt_volt, charge;
    datetime_t dt;

    rtc_get_datetime(&dt);
    printf("%02d.%02d.%d;%02d:%02d:%02d;", dt.day, dt.month, dt.year, dt.hour, dt.min, dt.sec);

    voltage = ina226_getBusVoltage();
    shunt_volt = ina226_getShuntVoltage();
    current = ina226_getCurrent();
    power = ina226_getPower();
    charge = battery_charge(voltage);
    printf("%.3f;%.3f;%.3f;%.6f;%.1f%%\n", voltage, current, power, shunt_volt, charge);

    return true;
}

bool await_usb_serial_str()
{
    int c = getchar_timeout_us(1);
    if (c != PICO_ERROR_TIMEOUT && buffer_index < BUFFER_LENGTH) {
        if (c == 13) {
            buffer[buffer_index++] = 0;
            return true;
        }
        buffer[buffer_index++] = (c & 0xFF);
    }
    if (buffer_index >= BUFFER_LENGTH)
    {
        buffer_index = 0;
    }
    
    return false;
}

int main()
{
    stdio_init_all();

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

    // I2C Initialisation. Using it at 400 kHz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    //uint16_t ret;
    //ret = ina226_getManufacturerID();
    //printf("%d", ret);

    if (ina226_reset()) {
        ina226_setAverage(INA226_4_SAMPLES);
        ina226_setBusVoltageConversionTime(INA226_332_us);
        ina226_setShuntVoltageConversionTime(INA226_1100_us);

        float shunt = 0.002292f;
        float current_LSB_mA = 1.0f;
        float current_zero_offset_mA = 0.0f;
        uint16_t bus_V_scaling_e4 = 10000;
        ina226_configure(shunt, current_LSB_mA, current_zero_offset_mA, bus_V_scaling_e4);

        struct repeating_timer timer;
        add_repeating_timer_ms(-1000, repeating_timer_callback, NULL, &timer);
    } 
    else
    {
        i2c_deinit(I2C_PORT);
        printf("Error: INA226 not found!");
    }

    while (true) {
        if (await_usb_serial_str()){
            uint8_t i;
            for (i = 0; i < buffer_index; i++) {
                printf("%c",buffer[i]);
            }
            buffer_index = 0;
            printf("\n");
        }
    }
}
