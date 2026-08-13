/*
   device configuration and setup data
   load/store device data from/to NVM
*/

/* --- Includes ------------------------------------------------------------ */
#include "esp_types.h"
#include "esp_err.h"

/* --- Defines ------------------------------------------------------------- */

#define DEVICE_NAME_MAX          25             // max. length of device name string (without \0)
#define WIFI_SSID_MAX            32             // max. length of WIFI SSID string (without \0)
#define WIFI_PASSWORD_MAX        64             // max. length of WIFI password string (without \0)


/* --- Types --------------------------------------------------------------- */

// output data set
typedef struct
{
   char     ac_DeviceName[DEVICE_NAME_MAX+1];      // device name string (max. 24 char + \0)
   char     ac_WiFiSSID[WIFI_SSID_MAX+1];          // WIFI SSID string (max. 32 char + \0)
   char     ac_WiFiPassword[WIFI_PASSWORD_MAX+1];  // WIFI password string (max. 64 char + \0)
} T_device_data;

/* --- Prototypes ---------------------------------------------------------- */

esp_err_t device_data_load(T_device_data * const opt_DeviceData);
esp_err_t device_data_store(const T_device_data * const opt_DeviceData);
