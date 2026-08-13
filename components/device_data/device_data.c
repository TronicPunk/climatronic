/*
   device configuration and setup data
   load/store device data from/to NVM
*/

/* --- Includes ------------------------------------------------------------ */
#include "device_data.h"
#include "nvm_data.h"
#include <string.h>

/* --- Defines ------------------------------------------------------------- */

/* --- Types --------------------------------------------------------------- */

// device data set
typedef struct
{
   uint32_t u32_CRC32;           // CRC32
   T_device_data t_DeviceData;   // device setup data
} T_device_data_crc;

_Static_assert(sizeof(T_device_data_crc) == NVM_DATA_DEVICE_SIZE, "T_device_data must be of size NVM_DATA_DEVICE_SIZE byte");

/* --- Module global variables --------------------------------------------- */

static const T_device_data mt_DeviceDataInvalid =
{
   .ac_DeviceName = "CLIMA_CONTROL",
   .ac_WiFiSSID = "WIFI_SSID_INVALID",
   .ac_WiFiPassword = "WIFI_PASSWORD_INVALID"
};

/* --- Module global prototypes -------------------------------------------- */

/* --- Implementation ------------------------------------------------------ */

/**
 * @brief load device setup data
 *
 * @param opt_DeviceData    copy device setup data to this location
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALID_CRC: CRC data error, return default values
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t device_data_load(T_device_data * const opt_DeviceData)
{
   esp_err_t t_Err;
   T_device_data_crc t_DeviceDataCrc;

   // read NVM data
   t_Err = nvm_data_load(NVM_DATA_DEVICE_ADDR, sizeof(T_device_data_crc), &t_DeviceDataCrc);

   if (t_Err == ESP_OK)
   {
      *opt_DeviceData = t_DeviceDataCrc.t_DeviceData;
   }
   else
   {
      *opt_DeviceData = mt_DeviceDataInvalid;
   }

   return t_Err;
}

/**
 * @brief store device setup data
 *
 * @param opt_DeviceData  device setup data to be stored
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALID_SIZE: Invalid data, not stored
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t device_data_store(const T_device_data * const opt_DeviceData)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_DeviceNameLen = strnlen(opt_DeviceData->ac_DeviceName, sizeof(opt_DeviceData->ac_DeviceName));
   const uint32_t u32_WiFiSSIDLen = strnlen(opt_DeviceData->ac_WiFiSSID, sizeof(opt_DeviceData->ac_WiFiSSID));
   const uint32_t u32_WiFiPWLen = strnlen(opt_DeviceData->ac_WiFiPassword, sizeof(opt_DeviceData->ac_WiFiPassword));

   // check device setup data
   if ((u32_DeviceNameLen > 0u) && (u32_DeviceNameLen <= DEVICE_NAME_MAX) &&
       (u32_WiFiSSIDLen > 0u) && (u32_WiFiSSIDLen <= WIFI_SSID_MAX) &&
       (u32_WiFiPWLen > 0u) && (u32_WiFiPWLen <= WIFI_PASSWORD_MAX))
   {
      T_device_data_crc t_DeviceDataCrc;
      T_device_data * const pt_DeviceData = &t_DeviceDataCrc.t_DeviceData;
      // copy setup data
      *pt_DeviceData = *opt_DeviceData;
      // fill unused string array data with \0
      (void)memset(&pt_DeviceData->ac_DeviceName[u32_DeviceNameLen+1u], 0, DEVICE_NAME_MAX-u32_DeviceNameLen);
      (void)memset(&pt_DeviceData->ac_WiFiSSID[u32_WiFiSSIDLen+1u], 0, DEVICE_NAME_MAX-u32_WiFiSSIDLen);
      (void)memset(&pt_DeviceData->ac_WiFiPassword[u32_WiFiPWLen+1u], 0, DEVICE_NAME_MAX-u32_WiFiPWLen);
      // store setup data
      t_Err = nvm_data_store(NVM_DATA_DEVICE_ADDR, sizeof(T_device_data_crc), &t_DeviceDataCrc);
   }
   else
   {
      t_Err = ESP_ERR_INVALID_SIZE;
   }

   return t_Err;
}
