//    FILE: INA226.c
//  AUTHOR: Vitaliy Zinoviev
// VERSION: 0.1
//    DATE: 2025-07-06
// PURPOSE: Library for INA226 power sensor
//     URL: 
//
//  Read the datasheet for the details

#include "pico/stdlib.h"
#include <math.h>
#include "hardware/i2c.h"
#include "INA226.h"

//  REGISTERS
#define INA226_CONFIGURATION              0x00
#define INA226_SHUNT_VOLTAGE              0x01
#define INA226_BUS_VOLTAGE                0x02
#define INA226_POWER                      0x03
#define INA226_CURRENT                    0x04
#define INA226_CALIBRATION                0x05
#define INA226_MASK_ENABLE                0x06
#define INA226_ALERT_LIMIT                0x07
#define INA226_MANUFACTURER               0xFE
#define INA226_DIE_ID                     0xFF


//  CONFIGURATION MASKS
#define INA226_CONF_RESET_MASK            0x8000
#define INA226_CONF_AVERAGE_MASK          0x0E00
#define INA226_CONF_BUSVC_MASK            0x01C0
#define INA226_CONF_SHUNTVC_MASK          0x0038
#define INA226_CONF_MODE_MASK             0x0007


float    _current_LSB;
float    _shunt;
float    _maxCurrent;
float    _current_zero_offset = 0;
uint16_t _bus_V_scaling_e4 = 10000;


uint16_t ina226_readRegister(uint8_t reg)
{
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, INA226_ADDR, &reg, 1, true);  // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, INA226_ADDR, buf, 2, false);  // false - finished with bus
    return buf[0] << 8 | buf[1];
}

int ina226_writeRegister(uint8_t reg, uint16_t value)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = value >> 8;
    buf[2] = value & 0xFF;
    int status = i2c_write_blocking(I2C_PORT, INA226_ADDR, buf, 3, false);
    return status;
}


////////////////////////////////////////////////////////
//
//  CORE FUNCTIONS
//

float ina226_getBusVoltage()
{
  uint16_t val = ina226_readRegister(INA226_BUS_VOLTAGE);
  float voltage = val * 1.25e-3;
  if (_bus_V_scaling_e4 != 10000)
  {
    voltage *= _bus_V_scaling_e4 * 1.0e-4;
  }
  return voltage;
}

float ina226_getShuntVoltage()
{
  int16_t val = ina226_readRegister(INA226_SHUNT_VOLTAGE);
  return val * 2.5e-6;   //  fixed 2.50 uV
}

float ina226_getCurrent()
{
  int16_t val = ina226_readRegister(INA226_CURRENT);
  return val * _current_LSB - _current_zero_offset;
}

float ina226_getPower()
{
  uint16_t val = ina226_readRegister(INA226_POWER);
  return val * (_current_LSB * 25);  //  fixed 25 Watt
}

bool ina226_isConversionReady()
{
  uint16_t mask = ina226_readRegister(INA226_MASK_ENABLE);
  return (mask & INA226_CONVERSION_READY_FLAG) == INA226_CONVERSION_READY_FLAG;
}

bool ina226_waitConversionReady(uint32_t timeout)
{
  while (!ina226_isConversionReady())
  {
    sleep_ms(1);
  }
  return true;
/*
  uint32_t start = millis();
  while ( (millis() - start) <= timeout)
  {
    if (ina226_isConversionReady()) return true;
    delay(1);  //  implicit yield();
  }
  return false;
*/
}


////////////////////////////////////////////////////////
//
//  CONFIGURATION
//

bool ina226_reset()
{
  uint16_t mask = ina226_readRegister(INA226_CONFIGURATION);
  mask |= INA226_CONF_RESET_MASK;
  int result = ina226_writeRegister(INA226_CONFIGURATION, mask);

  if (result < 0) return false;
  //  reset calibration
  _current_LSB = 0;
  _maxCurrent  = 0;
  _shunt       = 0;
  return true;
}

bool ina226_setAverage(uint8_t avg)
{
  if (avg > 7) return false;
  uint16_t mask = ina226_readRegister(INA226_CONFIGURATION);
  mask &= ~INA226_CONF_AVERAGE_MASK;
  mask |= (avg << 9);
  ina226_writeRegister(INA226_CONFIGURATION, mask);
  return true;
}

uint8_t ina226_getAverage()
{
  uint16_t mask = ina226_readRegister(INA226_CONFIGURATION);
  mask &= INA226_CONF_AVERAGE_MASK;
  mask >>= 9;
  return mask;
}

bool ina226_setBusVoltageConversionTime(uint8_t bvct)
{
  if (bvct > 7) return false;
  uint16_t mask = ina226_readRegister(INA226_CONFIGURATION);
  mask &= ~INA226_CONF_BUSVC_MASK;
  mask |= (bvct << 6);
  ina226_writeRegister(INA226_CONFIGURATION, mask);
  return true;
}

uint8_t ina226_getBusVoltageConversionTime()
{
  uint16_t mask = ina226_readRegister(INA226_CONFIGURATION);
  mask &= INA226_CONF_BUSVC_MASK;
  mask >>= 6;
  return mask;
}

bool ina226_setShuntVoltageConversionTime(uint8_t svct)
{
  if (svct > 7) return false;
  uint16_t mask = ina226_readRegister(INA226_CONFIGURATION);
  mask &= ~INA226_CONF_SHUNTVC_MASK;
  mask |= (svct << 3);
  ina226_writeRegister(INA226_CONFIGURATION, mask);
  return true;
}

uint8_t ina226_getShuntVoltageConversionTime()
{
  uint16_t mask = ina226_readRegister(INA226_CONFIGURATION);
  mask &= INA226_CONF_SHUNTVC_MASK;
  mask >>= 3;
  return mask;
}


////////////////////////////////////////////////////////
//
//  CALIBRATION
//

int ina226_setMaxCurrentShunt(float maxCurrent, float shunt, bool normalize)
{
  float shuntVoltage = maxCurrent * shunt;
  if (shuntVoltage > 0.08190)           return INA226_ERR_SHUNTVOLTAGE_HIGH;
  if (maxCurrent < 0.001)               return INA226_ERR_MAXCURRENT_LOW;
  if (shunt < INA226_MINIMAL_SHUNT_OHM) return INA226_ERR_SHUNT_LOW;

  _current_LSB = maxCurrent * 3.0517578125e-5;      //  maxCurrent / 32768;

  //  normalize the LSB to a round number
  //  LSB will increase
  if (normalize)
  {
    /*
       check if maxCurrent (normal) or shunt resistor
       (due to unusual low resistor values in relation to maxCurrent) determines currentLSB
       we have to take the upper value for currentLSB

       (adjusted in 0.6.0)
       calculation of currentLSB based on shunt resistor and calibration register limits (2 bytes)
       cal = 0.00512 / ( shunt * currentLSB )
       cal(max) = 2^15-1
       currentLSB(min) = 0.00512 / ( shunt * cal(max) )
       currentLSB(min) ~= 0.00512 / ( shunt * 2^15 )
       currentLSB(min) ~= 2^9 * 1e-5 / ( shunt * 2^15 )
       currentLSB(min) ~= 1e-5 / 2^6 / shunt
       currentLSB(min) ~= 1.5625e-7 / shunt
    */
    if ( 1.5625e-7 / shunt > _current_LSB ) {
      //  shunt resistor determines current_LSB
      //  => take this a starting point for current_LSB
      _current_LSB = 1.5625e-7 / shunt;
    }

    //  normalize _current_LSB to a value of 1, 2 or 5 * 1e-6 to 1e-3
    //  convert float to int
    uint16_t currentLSB_uA = (float)(_current_LSB * 1e+6);
    currentLSB_uA++;  //  ceil() would be more precise, but uses 176 bytes of flash.

    uint16_t factor = 1;  //  1uA to 1000uA
    uint8_t i = 0;        //  1 byte loop reduces footprint
    bool result = false;
    do {
      if ( 1 * factor >= currentLSB_uA) {
        _current_LSB = 1 * factor * 1e-6;
        result = true;
      } else if ( 2 * factor >= currentLSB_uA) {
        _current_LSB = 2 * factor * 1e-6;
        result = true;
      } else if ( 5 * factor >= currentLSB_uA) {
        _current_LSB = 5 * factor * 1e-6;
        result = true;
      } else {
        factor *= 10;
        i++;
      }
    } while ( (i < 4) && (!result) );  //  factor < 10000

    if (result == false)  //  not succeeded to normalize.
    {
      _current_LSB = 0;
      return INA226_ERR_NORMALIZE_FAILED;
    }
    // done
  }

  //  auto scale calibration if needed.
  uint32_t calib = round(0.00512 / (_current_LSB * shunt));
  while (calib > 32767)
  {
    _current_LSB *= 2;
    calib >>= 1;
  }
  ina226_writeRegister(INA226_CALIBRATION, calib);

  _maxCurrent = _current_LSB * 32768;
  _shunt = shunt;

  return INA226_ERR_NONE;

}

int ina226_configure(float shunt, float current_LSB_mA, float current_zero_offset_mA, uint16_t bus_V_scaling_e4)
{
  if (shunt < INA226_MINIMAL_SHUNT_OHM) return INA226_ERR_SHUNT_LOW;
  float maxCurrent = fmin((INA226_MAX_SHUNT_VOLTAGE / shunt), 32768 * current_LSB_mA * 1e-3);
  if (maxCurrent < 0.001)               return INA226_ERR_MAXCURRENT_LOW;

  _shunt               = shunt;
  _current_LSB         = current_LSB_mA * 1e-3;
  _current_zero_offset = current_zero_offset_mA * 1e-3;
  _bus_V_scaling_e4    = bus_V_scaling_e4;
  _maxCurrent          = maxCurrent;

  uint32_t calib = round(0.00512 / (_current_LSB * _shunt));
  ina226_writeRegister(INA226_CALIBRATION, calib);

  return INA226_ERR_NONE;
}


////////////////////////////////////////////////////////
//
//  OPERATING MODE
//

bool ina226_setMode(uint8_t mode)
{
  if (mode > 7) return false;
  uint16_t config = ina226_readRegister(INA226_CONFIGURATION);
  config &= ~INA226_CONF_MODE_MASK;
  config |= mode;
  ina226_writeRegister(INA226_CONFIGURATION, config);
  return true;
}


uint8_t ina226_getMode()
{
  uint16_t mode = ina226_readRegister(INA226_CONFIGURATION);
  mode &= INA226_CONF_MODE_MASK;
  return mode;
}


////////////////////////////////////////////////////////
//
//  ALERT
//
bool ina226_setAlertRegister(uint16_t mask)
{
  uint16_t result = ina226_writeRegister(INA226_MASK_ENABLE, (mask & 0xFC00));
  if (result != 0) return false;
  return true;
}


uint16_t ina226_getAlertFlag()
{
  return ina226_readRegister(INA226_MASK_ENABLE) & 0x001F;
}


bool ina226_setAlertLimit(uint16_t limit)
{
  uint16_t result = ina226_writeRegister(INA226_ALERT_LIMIT, limit);
  if (result != 0) return false;
  return true;
}


uint16_t ina226_getAlertLimit()
{
  return ina226_readRegister(INA226_ALERT_LIMIT);
}


////////////////////////////////////////////////////////
//
//  META INFORMATION
//
uint16_t ina226_getManufacturerID()
{
  return ina226_readRegister(INA226_MANUFACTURER);
}


uint16_t ina226_getDieID()
{
  return ina226_readRegister(INA226_DIE_ID);
}
