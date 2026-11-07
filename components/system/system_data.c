/*
   system configuration and setup data
   load/store system data from/to NVM
*/

/* --- Includes ------------------------------------------------------------ */
#include "system_data.h"
#include "nvm_data.h"
#include <string.h>

/* --- Defines ------------------------------------------------------------- */

/* --- Types --------------------------------------------------------------- */

typedef struct
{
   uint8_t au8_I2cBusFreq[I2C_BUS_COUNT]; // I2C bus frequency [I2C_BUS_FREQ_100K, I2C_BUS_FREQ_400K]
   uint8_t au8_Reserved[22];              // reserved NVM system data
} T_system_data;

// device data set
typedef struct
{
   uint32_t u32_CRC32;           // CRC32
   T_system_data t_SystemData;   // system setup data
} T_system_data_crc;

_Static_assert(sizeof(T_system_data_crc) == NVM_DATA_SYSTEM_SIZE, "T_system_data_crc must be of size NVM_DATA_DEVICE_SIZE byte");

/* --- Module global variables --------------------------------------------- */

static const T_system_data mt_SystemDataDefault =
{
   .au8_I2cBusFreq =
   {
      I2C_BUS_FREQ_100K, I2C_BUS_FREQ_100K, I2C_BUS_FREQ_100K,
      I2C_BUS_FREQ_100K, I2C_BUS_FREQ_100K, I2C_BUS_FREQ_100K
   },
   .au8_Reserved = { 0u },
};

static uint8_t mu8_NewData = 0u;
static T_system_data_crc mt_SystemDataCrc;

/* --- Module global prototypes -------------------------------------------- */

/* --- Implementation ------------------------------------------------------ */

/**
 * @brief load system setup data from NVM
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALID_CRC: CRC data error, return default values
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t system_data_load(void)
{
   esp_err_t t_Err;

   // read NVM data
   t_Err = nvm_data_load(NVM_DATA_SYSTEM_ADDR, sizeof(T_system_data_crc), &mt_SystemDataCrc);
   if (t_Err == ESP_ERR_INVALID_CRC)
   {
      esp_err_t t_ErrStore;

      mt_SystemDataCrc.t_SystemData = mt_SystemDataDefault;
      t_ErrStore = nvm_data_store(NVM_DATA_SYSTEM_ADDR, sizeof(T_system_data_crc), &mt_SystemDataCrc);
      t_Err = (t_ErrStore == ESP_FAIL) ? ESP_FAIL : t_Err;
   }

   if (t_Err == ESP_FAIL)
   {
      mt_SystemDataCrc.t_SystemData = mt_SystemDataDefault;
      mu8_NewData = 1u;
   }

   return t_Err;
}

/**
 * @brief store system setup data into NVM
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t system_data_store(void)
{
   if (mu8_NewData != 0u)
   {
      // store setup data
      esp_err_t t_Err = nvm_data_store(NVM_DATA_SYSTEM_ADDR, sizeof(T_system_data_crc), &mt_SystemDataCrc);
      if (t_Err == ESP_OK)
      {
         mu8_NewData = 0u;
      }
   }

   return (mu8_NewData != 0u) ? ESP_FAIL : ESP_OK;
}

/**
* Get i2c bus frequency for ou32_I2cBus. opu8_I2cFreq is I2C_BUS_FREQ_100K or I2C_BUS_FREQ_400K
*
* @return esp_err_t
*  - ESP_OK: Success
*  - ESP_ERR_INVALID_ARG: ou32_I2cBus out of range
*/
esp_err_t system_data_get_i2c_freq(const uint32_t ou32_I2cBus, uint8_t * const opu8_I2cFreq)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;

   if (ou32_I2cBus < I2C_BUS_COUNT)
   {
      *opu8_I2cFreq = mt_SystemDataCrc.t_SystemData.au8_I2cBusFreq[ou32_I2cBus];
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
* Set i2c bus frequency for ou32_I2cBus. opu8_I2cFreq must be I2C_BUS_FREQ_100K or I2C_BUS_FREQ_400K
*
* @return esp_err_t
*  - ESP_OK: Success
*  - ESP_ERR_INVALID_ARG: ou32_I2cBus or ou8_I2cFreq out of range
*/
esp_err_t system_data_set_i2c_freq(const uint32_t ou32_I2cBus, const uint8_t ou8_I2cFreq)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;

   if ((ou32_I2cBus < I2C_BUS_COUNT) && (ou8_I2cFreq <= I2C_BUS_FREQ_400K))
   {
      const uint8_t u8_I2cFreqOld = mt_SystemDataCrc.t_SystemData.au8_I2cBusFreq[ou32_I2cBus];
      if (u8_I2cFreqOld != ou8_I2cFreq)
      {
         mt_SystemDataCrc.t_SystemData.au8_I2cBusFreq[ou32_I2cBus] = ou8_I2cFreq;
         mu8_NewData = 1u;
      }

      t_Err = ESP_OK;
   }

   return t_Err;
}
