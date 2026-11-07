/*
   system configuration and setup data
   load/store system data from/to NVM
*/

#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

/* --- Includes ------------------------------------------------------------ */
#include "esp_types.h"
#include "esp_err.h"

/* --- Defines ------------------------------------------------------------- */

#define I2C_BUS_COUNT         6u    // number of available IIC sensor busses
#define I2C_BUS_FREQ_100K     0u    // IIC bus frequency = 100KBit/s
#define I2C_BUS_FREQ_400K     1u    // IIC bus frequency = 400KBit/s

/* --- Types --------------------------------------------------------------- */

/* --- Prototypes ---------------------------------------------------------- */

esp_err_t system_data_load(void);
esp_err_t system_data_store(void);
esp_err_t system_data_get_i2c_freq(const uint32_t ou32_I2cBus, uint8_t * const opu8_I2cFreq);
esp_err_t system_data_set_i2c_freq(const uint32_t ou32_I2cBus, const uint8_t ou8_I2cFreq);

#endif // SYSTEM_DATA_H