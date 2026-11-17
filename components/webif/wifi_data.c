/*
   WiFi configuration and setup data
   load/store WiFi data from/to NVM
*/

/* --- Includes ------------------------------------------------------------ */
#include "wifi_data.h"
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
} T_wifi_data_nvm;

typedef struct
{
   uint32_t u32_CRC32;           // CRC32
   T_wifi_data_nvm t_WiFiData;   // WiFi data (NVM layout)
} T_wifi_data_crc;

_Static_assert(sizeof(T_wifi_data_crc) == NVM_DATA_WIFI_SIZE, "T_wifi_data_nvm must be of size NVM_DATA_WIFI_SIZE byte");

/* --- Module global variables --------------------------------------------- */

static const T_wifi_data_nvm mt_WiFiDataDefault =
{
   .ac_HostName = "clima_control",
   .ac_WiFiSSID = "WIFI_SSID_INVALID",
   .au8_BSSID = { 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u },
   .ac_WiFiPw = "WIFI_PASSWORD_INVALID",
   .au8_Reserved = { 0x00u },
};

static uint8_t mu8_NewData = 0u;
static T_wifi_data_crc mt_WiFiDataCrc;

/* --- Module global prototypes -------------------------------------------- */

/* --- Implementation ------------------------------------------------------ */

/**
 * @brief load WiFi setup data from NVM
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALID_CRC: CRC data error, return default values
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t wifi_data_load(void)
{
   esp_err_t t_Err;

   // read NVM data
   t_Err = nvm_data_load(NVM_DATA_WIFI_ADDR, sizeof(T_wifi_data_crc), &mt_WiFiDataCrc);
   if (t_Err == ESP_ERR_INVALID_CRC)
   {
      esp_err_t t_ErrStore;

      mt_WiFiDataCrc.t_WiFiData = mt_WiFiDataDefault;
      t_ErrStore = nvm_data_store(NVM_DATA_WIFI_ADDR, sizeof(T_wifi_data_crc), &mt_WiFiDataCrc);
      t_Err = (t_ErrStore == ESP_FAIL) ? ESP_FAIL : t_Err;
   }

   if (t_Err == ESP_FAIL)
   {
      mt_WiFiDataCrc.t_WiFiData = mt_WiFiDataDefault;
      mu8_NewData = 1u;
   }

   return t_Err;
}

/**
 * @brief store WiFi setup data
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t wifi_data_store(void)
{
   if (mu8_NewData != 0u)
   {
      // store setup data
      esp_err_t t_Err = nvm_data_store(NVM_DATA_WIFI_ADDR, sizeof(T_wifi_data_crc), &mt_WiFiDataCrc);
      if (t_Err == ESP_OK)
      {
         mu8_NewData = 0u;
      }
   }

   return (mu8_NewData != 0u) ? ESP_FAIL : ESP_OK;
}

/**
 * @brief get WiFi data
 */
void wifi_data_get(T_wifi_data * const opt_WiFiData)
{
   T_wifi_data_nvm * const pt_WiFiData = &mt_WiFiDataCrc.t_WiFiData;

   opt_WiFiData->ac_HostName = pt_WiFiData->ac_HostName;
   opt_WiFiData->ac_WiFiSSID = pt_WiFiData->ac_WiFiSSID;
   opt_WiFiData->au8_BSSID = pt_WiFiData->au8_BSSID;
   opt_WiFiData->ac_WiFiPw = pt_WiFiData->ac_WiFiPw;
}

/**
 * @brief set device host name
 */
esp_err_t wifi_data_set_hostname(const char * const oac_HostName)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_HostNameLen = strnlen(oac_HostName, DEVICE_HOST_NAME_MAX + 1u);

   if ((u32_HostNameLen > 0u) && (u32_HostNameLen <= DEVICE_HOST_NAME_MAX))
   {
      T_wifi_data_nvm * pt_WiFiData = &mt_WiFiDataCrc.t_WiFiData;
      if (strncmp(pt_WiFiData->ac_HostName, oac_HostName, DEVICE_HOST_NAME_MAX) != 0)
      {
         (void)strncpy(pt_WiFiData->ac_HostName, oac_HostName, DEVICE_HOST_NAME_MAX);
         pt_WiFiData->ac_HostName[DEVICE_HOST_NAME_MAX] = '\0';
         mu8_NewData = 1u; // set new data flag
      }
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief set WiFi SSID
 */
esp_err_t wifi_data_set_ssid(const char * const oac_SSID)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_SSIDLen = strnlen(oac_SSID, WIFI_SSID_MAX + 1u);

   if (u32_SSIDLen <= WIFI_SSID_MAX)
   {
      T_wifi_data_nvm * pt_WiFiData = &mt_WiFiDataCrc.t_WiFiData;
      if (strncmp(pt_WiFiData->ac_WiFiSSID, oac_SSID, WIFI_SSID_MAX) != 0)
      {
         (void)strncpy(pt_WiFiData->ac_WiFiSSID, oac_SSID, WIFI_SSID_MAX);
         pt_WiFiData->ac_WiFiSSID[WIFI_SSID_MAX] = '\0';
         mu8_NewData = 1u; // set new data flag
      }
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief set WIFI BSSID
 */
esp_err_t wifi_data_set_bssid(const uint8_t * const oau8_BSSID)
{
   T_wifi_data_nvm * pt_WiFiData = &mt_WiFiDataCrc.t_WiFiData;

   if (memcmp(pt_WiFiData->au8_BSSID, oau8_BSSID, WIFI_BSSID_SIZE) != 0)
   {
      memcpy(pt_WiFiData->au8_BSSID, oau8_BSSID, WIFI_BSSID_SIZE);
      mu8_NewData = 1u;
   }

   return ESP_OK;
}

/**
 * @brief set WIFI PW
 */
esp_err_t wifi_data_set_wifipw(const char * const oac_WiFiPw)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_WiFiPwLen = strnlen(oac_WiFiPw, WIFI_PASSWORD_MAX + 1u);

   if (u32_WiFiPwLen <= WIFI_PASSWORD_MAX)
   {
      T_wifi_data_nvm * pt_WiFiData = &mt_WiFiDataCrc.t_WiFiData;
      if (strncmp(pt_WiFiData->ac_WiFiPw, oac_WiFiPw, WIFI_PASSWORD_MAX) != 0)
      {
         (void)strncpy(pt_WiFiData->ac_WiFiPw, oac_WiFiPw, WIFI_PASSWORD_MAX);
         pt_WiFiData->ac_WiFiPw[WIFI_PASSWORD_MAX] = '\0';
         mu8_NewData = 1u; // set new data flag
      }
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief check if any data in buffer opu8_Data is not zero
 */
bool wifi_data_not_zero(const uint8_t * const opu8_Data, const uint32_t ou32_Size)
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