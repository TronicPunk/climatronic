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

typedef struct
{
   char     ac_HostName[DEVICE_HOST_NAME_MAX+1];   // device host name string (max. 24 char + \0)
   char     ac_WiFiSSID[WIFI_SSID_MAX+1];          // WIFI SSID string (max. 32 char + \0)
   uint8_t  au8_BSSID[WIFI_BSSID_SIZE];            // WIFI AP BSSID (MAC address)
   char     ac_WiFiPw[WIFI_PASSWORD_MAX+1];        // WIFI password string (max. 63 char + \0)
   uint8_t  au8_Reserved[27];                      // reserved space (fill 32byte page size)
} T_device_data_nvm;

typedef struct
{
   uint32_t u32_CRC32;              // CRC32
   T_device_data_nvm t_DeviceData;  // device data (NVM layout)
} T_device_data_crc;

_Static_assert(sizeof(T_device_data_crc) == NVM_DATA_DEVICE_SIZE, "T_device_data_nvm must be of size NVM_DATA_DEVICE_SIZE byte");

/* --- Module global variables --------------------------------------------- */

static const T_device_data_nvm mt_DeviceDataDefault =
{
   .ac_HostName = "clima_control",
   .ac_WiFiSSID = "WIFI_SSID_INVALID",
   .au8_BSSID = { 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u },
   .ac_WiFiPw = "WIFI_PASSWORD_INVALID",
   .au8_Reserved = { 0x00u },
};

static uint8_t mu8_NewData = 0u;
static T_device_data_crc mt_DeviceDataCrc;

/* --- Module global prototypes -------------------------------------------- */

/* --- Implementation ------------------------------------------------------ */

/**
 * @brief load device setup data from NVM
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALID_CRC: CRC data error, return default values
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t device_data_load(void)
{
   esp_err_t t_Err;

   // read NVM data
   t_Err = nvm_data_load(NVM_DATA_DEVICE_ADDR, sizeof(T_device_data_crc), &mt_DeviceDataCrc);
   if (t_Err == ESP_ERR_INVALID_CRC)
   {
      esp_err_t t_ErrStore;

      mt_DeviceDataCrc.t_DeviceData = mt_DeviceDataDefault;
      t_ErrStore = nvm_data_store(NVM_DATA_DEVICE_ADDR, sizeof(T_device_data_crc), &mt_DeviceDataCrc);
      t_Err = (t_ErrStore == ESP_FAIL) ? ESP_FAIL : t_Err;
   }

   if (t_Err == ESP_FAIL)
   {
      mt_DeviceDataCrc.t_DeviceData = mt_DeviceDataDefault;
      mu8_NewData = 1u;
   }

   return t_Err;
}

/**
 * @brief store device setup data
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t device_data_store(void)
{
   if (mu8_NewData != 0u)
   {
      // store setup data
      esp_err_t t_Err = nvm_data_store(NVM_DATA_DEVICE_ADDR, sizeof(T_device_data_crc), &mt_DeviceDataCrc);
      if (t_Err == ESP_OK)
      {
         mu8_NewData = 0u;
      }
   }

   return (mu8_NewData != 0u) ? ESP_FAIL : ESP_OK;
}

/**
 * @brief get device data
 */
void device_data_get(T_device_data * const opt_DeviceData)
{
   T_device_data_nvm * const pt_DeviceData = &mt_DeviceDataCrc.t_DeviceData;

   opt_DeviceData->ac_HostName = pt_DeviceData->ac_HostName;
   opt_DeviceData->ac_WiFiSSID = pt_DeviceData->ac_WiFiSSID;
   opt_DeviceData->au8_BSSID = pt_DeviceData->au8_BSSID;
   opt_DeviceData->ac_WiFiPw = pt_DeviceData->ac_WiFiPw;
}

/**
 * @brief set device host name
 */
esp_err_t device_data_set_hostname(const char * const oac_HostName)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_HostNameLen = strnlen(oac_HostName, DEVICE_HOST_NAME_MAX + 1u);

   if ((u32_HostNameLen > 0u) && (u32_HostNameLen <= DEVICE_HOST_NAME_MAX))
   {
      T_device_data_nvm * pt_DeviceData = &mt_DeviceDataCrc.t_DeviceData;
      if (strncmp(pt_DeviceData->ac_HostName, oac_HostName, DEVICE_HOST_NAME_MAX) != 0)
      {
         (void)strncpy(pt_DeviceData->ac_HostName, oac_HostName, DEVICE_HOST_NAME_MAX);
         pt_DeviceData->ac_HostName[DEVICE_HOST_NAME_MAX] = '\0';
         mu8_NewData = 1u; // set new data flag
      }
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief set device SSID
 */
esp_err_t device_data_set_ssid(const char * const oac_SSID)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_SSIDLen = strnlen(oac_SSID, WIFI_SSID_MAX + 1u);

   if (u32_SSIDLen <= WIFI_SSID_MAX)
   {
      T_device_data_nvm * pt_DeviceData = &mt_DeviceDataCrc.t_DeviceData;
      if (strncmp(pt_DeviceData->ac_WiFiSSID, oac_SSID, WIFI_SSID_MAX) != 0)
      {
         (void)strncpy(pt_DeviceData->ac_WiFiSSID, oac_SSID, WIFI_SSID_MAX);
         pt_DeviceData->ac_WiFiSSID[WIFI_SSID_MAX] = '\0';
         mu8_NewData = 1u; // set new data flag
      }
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief set device WIFI BSSID
 */
esp_err_t device_data_set_bssid(const uint8_t * const oau8_BSSID)
{
   T_device_data_nvm * pt_DeviceData = &mt_DeviceDataCrc.t_DeviceData;

   if (memcmp(pt_DeviceData->au8_BSSID, oau8_BSSID, WIFI_BSSID_SIZE) != 0)
   {
      memcpy(pt_DeviceData->au8_BSSID, oau8_BSSID, WIFI_BSSID_SIZE);
      mu8_NewData = 1u;
   }

   return ESP_OK;
}

/**
 * @brief set device WIFI PW
 */
esp_err_t device_data_set_wifipw(const char * const oac_WiFiPw)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_WiFiPwLen = strnlen(oac_WiFiPw, WIFI_PASSWORD_MAX + 1u);

   if (u32_WiFiPwLen <= WIFI_PASSWORD_MAX)
   {
      T_device_data_nvm * pt_DeviceData = &mt_DeviceDataCrc.t_DeviceData;
      if (strncmp(pt_DeviceData->ac_WiFiPw, oac_WiFiPw, WIFI_PASSWORD_MAX) != 0)
      {
         (void)strncpy(pt_DeviceData->ac_WiFiPw, oac_WiFiPw, WIFI_PASSWORD_MAX);
         pt_DeviceData->ac_WiFiPw[WIFI_PASSWORD_MAX] = '\0';
         mu8_NewData = 1u; // set new data flag
      }
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief check if any data in buffer opu8_Data is not zero
 */
bool device_data_not_zero(const uint8_t * const opu8_Data, const uint32_t ou32_Size)
{
   bool q_DataNotZero = false;

   for (uint32_t i = 0; i < ou32_Size; i++)
   {
      if (opu8_Data[i] != 0u)
      {
         q_DataNotZero = true;
         break;
      }
   }

   return q_DataNotZero;
}