/*
   WiFi configuration and setup data
   load/store WiFi data from/to NVM
*/

#ifndef WIFI_DATA_H
#define WIFI_DATA_H

/* --- Includes ------------------------------------------------------------ */
#include "esp_types.h"
#include "esp_err.h"

/* --- Defines ------------------------------------------------------------- */

#define DEVICE_HOST_NAME_MAX     25    // max. length of device name string (without \0)
#define WIFI_SSID_MAX            32    // max. length of WIFI SSID string (without \0)
#define WIFI_BSSID_SIZE          6     // AP BSSID size (AP MAC address)
#define WIFI_PASSWORD_MAX        63    // max. length of WIFI password string (without \0)


/* --- Types --------------------------------------------------------------- */

// WiFi data set
typedef struct
{
   char * ac_HostName;     // device host name string
   char * ac_WiFiSSID;     // WIFI SSID string
   uint8_t * au8_BSSID;    // WIFI AP BSSID (MAC address)
   char * ac_WiFiPw;       // WIFI password string
} T_wifi_data;

/* --- Prototypes ---------------------------------------------------------- */

esp_err_t wifi_data_load(void);
esp_err_t wifi_data_store(void);
void      wifi_data_get(T_wifi_data * const opt_WiFiData);
esp_err_t wifi_data_set_hostname(const char * const oac_HostName);
esp_err_t wifi_data_set_ssid(const char * const oac_SSID);
esp_err_t wifi_data_set_bssid(const uint8_t * const oau8_BSSID);
esp_err_t wifi_data_set_wifipw(const char * const oac_WiFiPw);
bool      wifi_data_not_zero(const uint8_t * const opu8_Data, const uint32_t ou32_Size);

#endif // WIFI_DATA_H