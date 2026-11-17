/*
   MQTT configuration and setup data
   load/store MQTT data from/to NVM
*/

#ifndef MQTT_DATA_H
#define MQTT_DATA_H

/* --- Includes ------------------------------------------------------------ */
#include "esp_types.h"
#include "esp_err.h"

/* --- Defines ------------------------------------------------------------- */

#define MQTT_BROKER_URI_MAX         127   // max. length of broker URI (without \0)
#define MQTT_BROKER_USER_NAME_MAX   31    // max. length of broker user name (without \0)
#define MQTT_BROKER_USER_PW_MAX     31    // max. length of broker user password (without \0)
#define MQTT_TOPIC_PREFIX_MAX       31    // max. length of MQTT topic prefix (without \0)

/* --- Types --------------------------------------------------------------- */

// MQTT data set
typedef struct
{
   uint32_t u32_PublishIntervalMs;  // sensor and output data publish interval in milli seconds
   uint32_t u32_OutSetTimeoutMs;    // output set timeout in milli seconds [500 ... 60000]
   uint8_t u8_Reserved[20];         // reserved
} T_mqtt_setup;

typedef struct
{
   T_mqtt_setup * pt_MqttSetup;     // MQTT setup data (interval time, timout, ...)
   char * ac_BrokerUri;             // MQTT broker URI
   char * ac_BrokerUserName;        // MQTT broker user name
   char * ac_BrokerUserPw;          // MQTT broker user password
   char * ac_TopicPrefix;           // MQTT topic prefix
} T_mqtt_data;

/* --- Prototypes ---------------------------------------------------------- */

esp_err_t mqtt_data_load(void);
esp_err_t mqtt_data_store(void);
void      mqtt_data_get(T_mqtt_data * const opt_MqttData);
esp_err_t mqtt_data_set_broker_uri(const char * const oac_BrokerUri);
esp_err_t mqtt_data_set_broker_username(const char * const oac_BrokerUserName);
esp_err_t mqtt_data_set_broker_userpw(const char * const oac_BrokerUserPw);
esp_err_t mqtt_data_set_topic_prefix(const char * const oac_TopicPrefix);
esp_err_t mqtt_data_set_publish_interval(const uint32_t ou32_PublishIntervalMs);
esp_err_t mqtt_data_set_outset_timeout(const uint32_t ou32_OutSetTimeoutMs);

#endif // MQTT_DATA_H