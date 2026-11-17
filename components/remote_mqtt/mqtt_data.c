/*
   MQTT configuration and setup data
   load/store MQTT data from/to NVM
*/

/* --- Includes ------------------------------------------------------------ */
#include "mqtt_data.h"
#include "nvm_data.h"
#include <string.h>

/* --- Defines ------------------------------------------------------------- */

/* --- Types --------------------------------------------------------------- */

typedef struct
{
   T_mqtt_setup t_MqttSetup;                             // MQTT setup data (interval time, timout, ...)
   char ac_BrokerUri[MQTT_BROKER_URI_MAX+1];             // MQTT broker URI
   char ac_BrokerUserName[MQTT_BROKER_USER_NAME_MAX+1];  // MQTT broker user name
   char ac_BrokerUserPw[MQTT_BROKER_USER_PW_MAX+1];      // MQTT broker user password
   char ac_TopicPrefix[MQTT_TOPIC_PREFIX_MAX+1];         // MQTT topic prefix
} T_mqtt_data_nvm;

typedef struct
{
   uint32_t u32_CRC32;           // CRC32
   T_mqtt_data_nvm t_MqttData;   // MQTT data (NVM layout)
} T_mqtt_data_crc;

_Static_assert(sizeof(T_mqtt_data_crc) == NVM_DATA_MQTT_SIZE, "T_mqtt_data_nvm must be of size NVM_DATA_MQTT_SIZE byte");

/* --- Module global variables --------------------------------------------- */

static const T_mqtt_data_nvm mt_MqttDataDefault =
{
   .t_MqttSetup =
   {
      .u32_PublishIntervalMs = 4000u,
      .u32_OutSetTimeoutMs = 4000u,
      .u8_Reserved = { 0x00u },
   },
   .ac_BrokerUri = "",
   .ac_BrokerUserName = "",
   .ac_BrokerUserPw = "",
   .ac_TopicPrefix = "climacontrol",
};

static uint8_t mu8_NewData = 0u;
static T_mqtt_data_crc mt_MqttDataCrc;

/* --- Module global prototypes -------------------------------------------- */

/* --- Implementation ------------------------------------------------------ */

/**
 * @brief load MQTT setup data from NVM
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALID_CRC: CRC data error, return default values
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t mqtt_data_load(void)
{
   esp_err_t t_Err;

   // read NVM data
   t_Err = nvm_data_load(NVM_DATA_MQTT_ADDR, sizeof(T_mqtt_data_crc), &mt_MqttDataCrc);
   if (t_Err == ESP_ERR_INVALID_CRC)
   {
      esp_err_t t_ErrStore;

      mt_MqttDataCrc.t_MqttData = mt_MqttDataDefault;
      t_ErrStore = nvm_data_store(NVM_DATA_MQTT_ADDR, sizeof(T_mqtt_data_crc), &mt_MqttDataCrc);
      t_Err = (t_ErrStore == ESP_FAIL) ? ESP_FAIL : t_Err;
   }

   if (t_Err == ESP_FAIL)
   {
      mt_MqttDataCrc.t_MqttData = mt_MqttDataDefault;
      mu8_NewData = 1u;
   }

   return t_Err;
}

/**
 * @brief store MQTT setup data
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t mqtt_data_store(void)
{
   if (mu8_NewData != 0u)
   {
      // store setup data
      esp_err_t t_Err = nvm_data_store(NVM_DATA_MQTT_ADDR, sizeof(T_mqtt_data_crc), &mt_MqttDataCrc);
      if (t_Err == ESP_OK)
      {
         mu8_NewData = 0u;
      }
   }

   return (mu8_NewData != 0u) ? ESP_FAIL : ESP_OK;
}

/**
 * @brief get MQTT data
 */
void mqtt_data_get(T_mqtt_data * const opt_MqttData)
{
   T_mqtt_data_nvm * const pt_MqttData = &mt_MqttDataCrc.t_MqttData;

   opt_MqttData->pt_MqttSetup = &pt_MqttData->t_MqttSetup;
   opt_MqttData->ac_BrokerUri = pt_MqttData->ac_BrokerUri;
   opt_MqttData->ac_BrokerUserName = pt_MqttData->ac_BrokerUserName;
   opt_MqttData->ac_BrokerUserPw = pt_MqttData->ac_BrokerUserPw;
   opt_MqttData->ac_TopicPrefix = pt_MqttData->ac_TopicPrefix;
}

/**
 * @brief set MQTT broker URI
 */
esp_err_t mqtt_data_set_broker_uri(const char * const oac_BrokerUri)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_BrokerUriLen = strnlen(oac_BrokerUri, MQTT_BROKER_URI_MAX + 1u);

   if (u32_BrokerUriLen <= MQTT_BROKER_URI_MAX)
   {
      T_mqtt_data_nvm * pt_MqttData = &mt_MqttDataCrc.t_MqttData;
      if (strncmp(pt_MqttData->ac_BrokerUri, oac_BrokerUri, MQTT_BROKER_URI_MAX) != 0)
      {
         (void)strncpy(pt_MqttData->ac_BrokerUri, oac_BrokerUri, MQTT_BROKER_URI_MAX);
         pt_MqttData->ac_BrokerUri[MQTT_BROKER_URI_MAX] = '\0';
         mu8_NewData = 1u; // set new data flag
      }
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief set MQTT broker user name
 */
esp_err_t mqtt_data_set_broker_username(const char * const oac_BrokerUserName)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_UserNameLen = strnlen(oac_BrokerUserName, MQTT_BROKER_USER_NAME_MAX + 1u);

   if (u32_UserNameLen <= MQTT_BROKER_USER_NAME_MAX)
   {
      T_mqtt_data_nvm * pt_MqttData = &mt_MqttDataCrc.t_MqttData;
      if (strncmp(pt_MqttData->ac_BrokerUserName, oac_BrokerUserName, MQTT_BROKER_USER_NAME_MAX) != 0)
      {
         (void)strncpy(pt_MqttData->ac_BrokerUserName, oac_BrokerUserName, MQTT_BROKER_USER_NAME_MAX);
         pt_MqttData->ac_BrokerUserName[MQTT_BROKER_USER_NAME_MAX] = '\0';
         mu8_NewData = 1u; // set new data flag
      }
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief set MQTT broker user password
 */
esp_err_t mqtt_data_set_broker_userpw(const char * const oac_BrokerUserPw)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_UserPwLen = strnlen(oac_BrokerUserPw, MQTT_BROKER_USER_PW_MAX + 1u);

   if (u32_UserPwLen <= MQTT_BROKER_USER_PW_MAX)
   {
      T_mqtt_data_nvm * pt_MqttData = &mt_MqttDataCrc.t_MqttData;
      if (strncmp(pt_MqttData->ac_BrokerUserPw, oac_BrokerUserPw, MQTT_BROKER_USER_PW_MAX) != 0)
      {
         (void)strncpy(pt_MqttData->ac_BrokerUserPw, oac_BrokerUserPw, MQTT_BROKER_USER_PW_MAX);
         pt_MqttData->ac_BrokerUserPw[MQTT_BROKER_USER_PW_MAX] = '\0';
         mu8_NewData = 1u; // set new data flag
      }
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief set MQTT topic prefix
 */
esp_err_t mqtt_data_set_topic_prefix(const char * const oac_TopicPrefix)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_MqttPrefixLen = strnlen(oac_TopicPrefix, MQTT_TOPIC_PREFIX_MAX + 1u);

   if (u32_MqttPrefixLen <= MQTT_TOPIC_PREFIX_MAX)
   {
      T_mqtt_data_nvm * pt_MqttData = &mt_MqttDataCrc.t_MqttData;
      if (strncmp(pt_MqttData->ac_TopicPrefix, oac_TopicPrefix, MQTT_TOPIC_PREFIX_MAX) != 0)
      {
         (void)strncpy(pt_MqttData->ac_TopicPrefix, oac_TopicPrefix, MQTT_TOPIC_PREFIX_MAX);
         pt_MqttData->ac_TopicPrefix[MQTT_TOPIC_PREFIX_MAX] = '\0';
         mu8_NewData = 1u; // set new data flag
      }
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief set MQTT publish interval time in milli seconds. Valid range is [500 ... 60000]
 */
esp_err_t mqtt_data_set_publish_interval(const uint32_t ou32_PublishIntervalMs)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   if ((ou32_PublishIntervalMs >= 500uL) && (ou32_PublishIntervalMs <= 60000uL))
   {
      T_mqtt_setup * pt_MqttSetup = &mt_MqttDataCrc.t_MqttData.t_MqttSetup;

      if (pt_MqttSetup->u32_PublishIntervalMs != ou32_PublishIntervalMs)
      {
         pt_MqttSetup->u32_PublishIntervalMs = ou32_PublishIntervalMs;
         mu8_NewData = 1u; // set new data flag
      }

      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * @brief set MQTT output set timeout in milli seconds. Valid range is [500 ... max]
 */
esp_err_t mqtt_data_set_outset_timeout(const uint32_t ou32_OutSetTimeoutMs)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   if (ou32_OutSetTimeoutMs >= 500uL)
   {
      T_mqtt_setup * pt_MqttSetup = &mt_MqttDataCrc.t_MqttData.t_MqttSetup;

      if (pt_MqttSetup->u32_OutSetTimeoutMs != ou32_OutSetTimeoutMs)
      {
         pt_MqttSetup->u32_OutSetTimeoutMs = ou32_OutSetTimeoutMs;
         mu8_NewData = 1u; // set new data flag
      }

      t_Err = ESP_OK;
   }

   return t_Err;
}
