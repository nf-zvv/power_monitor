//    FILE: INA226.h
//  AUTHOR: Vitaliy Zinoviev
// VERSION: 0.1
//    DATE: 2025-07-06
// PURPOSE: Library for INA226 power sensor
//     URL: 
//
//  Read the datasheet for the details

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9


// By default these devices  are on bus address 0x40
static int INA226_ADDR = 0x40;

//  set by setAlertRegister
#define INA226_SHUNT_OVER_VOLTAGE         0x8000
#define INA226_SHUNT_UNDER_VOLTAGE        0x4000
#define INA226_BUS_OVER_VOLTAGE           0x2000
#define INA226_BUS_UNDER_VOLTAGE          0x1000
#define INA226_POWER_OVER_LIMIT           0x0800
#define INA226_CONVERSION_READY           0x0400


//  returned by getAlertFlag
#define INA226_ALERT_FUNCTION_FLAG        0x0010
#define INA226_CONVERSION_READY_FLAG      0x0008
#define INA226_MATH_OVERFLOW_FLAG         0x0004
#define INA226_ALERT_POLARITY_FLAG        0x0002
#define INA226_ALERT_LATCH_ENABLE_FLAG    0x0001


//  returned by setMaxCurrentShunt
#define INA226_ERR_NONE                   0x0000
#define INA226_ERR_SHUNTVOLTAGE_HIGH      0x8000
#define INA226_ERR_MAXCURRENT_LOW         0x8001
#define INA226_ERR_SHUNT_LOW              0x8002
#define INA226_ERR_NORMALIZE_FAILED       0x8003

//  See issue #26
#define INA226_MINIMAL_SHUNT_OHM          0.001

#define INA226_MAX_WAIT_MS                600   //  millis

#define INA226_MAX_SHUNT_VOLTAGE          (81.92 / 1000)


//  for setAverage() and getAverage()
enum ina226_average_enum {
    INA226_1_SAMPLE     = 0,
    INA226_4_SAMPLES    = 1,
    INA226_16_SAMPLES   = 2,
    INA226_64_SAMPLES   = 3,
    INA226_128_SAMPLES  = 4,
    INA226_256_SAMPLES  = 5,
    INA226_512_SAMPLES  = 6,
    INA226_1024_SAMPLES = 7
};


//  for BVCT and SVCT conversion timing.
enum ina226_timing_enum {
    INA226_140_us  = 0,
    INA226_204_us  = 1,
    INA226_332_us  = 2,
    INA226_588_us  = 3,
    INA226_1100_us = 4,
    INA226_2100_us = 5,
    INA226_4200_us = 6,
    INA226_8300_us = 7
};


float ina226_getBusVoltage();
float ina226_getShuntVoltage();
float ina226_getCurrent();
float ina226_getPower();

bool ina226_reset();
bool ina226_setAverage(uint8_t);
uint8_t ina226_getAverage();
bool ina226_setBusVoltageConversionTime(uint8_t);
uint8_t ina226_getBusVoltageConversionTime();
bool ina226_setShuntVoltageConversionTime(uint8_t);
uint8_t ina226_getShuntVoltageConversionTime();

int ina226_setMaxCurrentShunt(float, float, bool);
int ina226_configure(float, float, float, uint16_t);

bool ina226_setMode(uint8_t);
uint8_t ina226_getMode();

bool ina226_setAlertRegister(uint16_t);
uint16_t ina226_getAlertFlag();
bool ina226_setAlertLimit(uint16_t);
uint16_t ina226_getAlertLimit();

uint16_t ina226_getManufacturerID();
uint16_t ina226_getDieID();
