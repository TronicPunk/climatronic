
#include "mqtt_interface.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "wifi_connect.h"
#include "output.h"
#include "sensor.h"
#include "sensor_api.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "timer.h"
#include "inout.h"

#define MQTT_TOPIC_MAX_LEN 128u
#define MQTT_PAYLOAD_MAX_LEN 256u

static const char *TAG = "mqtt_client";
static esp_mqtt_client_handle_t mqtt_client;
static volatile bool mqtt_connected;
static T_mqtt_data mt_MqttDataRuntime;
static T_mqtt_data *mpt_MqttData;
static TaskHandle_t mt_PublishTaskHandle;
static char * mpc_TopicPrefix;                           // MQTT_TOPIC_PREFIX
static char mac_StatusTopic[MQTT_TOPIC_PREFIX_MAX +8];   // MQTT_TOPIC_PREFIX + "/status" + '\0'

static void m_publish_state(void);
static void m_publish_message(const char *topic, const char *payload,
                              const int payload_len, const int retain);
static void m_mqtt_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data);
static void m_mqtt_publish_task(void *arg);
static void m_subscribe_output_commands(void);
static void m_handle_output_command(const char *topic, const size_t topic_len,
                                    const char *data, const size_t data_len);

static int m_format_float(char *buffer, const size_t size, const float value)
{
   if (!isfinite(value) || (value == SENSOR_VALUE_INVALID))
   {
      return snprintf(buffer, size, "null");
   }
   return snprintf(buffer, size, "%.2f", (double)value);
}

static void m_publish_message(const char *topic, const char *payload,
                             const int payload_len, const int retain)
{
   if (esp_mqtt_client_publish(mqtt_client, topic, payload, payload_len, 1, retain) < 0)
   {
      ESP_LOGW(TAG, "Failed to queue MQTT message for %s", topic);
   }
}

static void m_publish_sensor(const uint16_t sensor_idx)
{
   // requires 504 byte stack
   T_sensor_info info;
   T_sensor_data data;
   char topic[MQTT_TOPIC_MAX_LEN];
   char payload[MQTT_PAYLOAD_MAX_LEN];
   char temperature[16];
   char pressure[16];
   char humidity[16];
   char co2[16];

   if ((sensor_get_info(sensor_idx, &info) != ESP_OK) ||
       ((info.u8_Flags & SENSOR_FLAGS_REMOTE_RD) == 0u) ||
       (sensor_get_data(sensor_idx, &data) != ESP_OK))
   {
      return;
   }

   const int topic_len = snprintf(topic, sizeof(topic), "%s/sensors/%s/state", mpc_TopicPrefix, info.s_Name);
   const int temperature_len = m_format_float(temperature, sizeof(temperature), data.f32_Temperature);
   const int pressure_len = m_format_float(pressure, sizeof(pressure), data.f32_Pressure);
   const int humidity_len = m_format_float(humidity, sizeof(humidity), data.f32_Humidity);
   const int co2_len = m_format_float(co2, sizeof(co2), data.f32_CO2);
   if ((topic_len < 0) || ((size_t)topic_len >= sizeof(topic)) ||
       (temperature_len < 0) || ((size_t)temperature_len >= sizeof(temperature)) ||
       (pressure_len < 0) || ((size_t)pressure_len >= sizeof(pressure)) ||
       (humidity_len < 0) || ((size_t)humidity_len >= sizeof(humidity)) ||
       (co2_len < 0) || ((size_t)co2_len >= sizeof(co2)))
   {
      ESP_LOGW(TAG, "Sensor %u MQTT state exceeds buffer capacity", (unsigned int)sensor_idx);
      return;
   }
   const uint32_t u32_AgeSec = get_time_sec() - data.u32_TimeStampSec;
   const int payload_len = snprintf(payload, sizeof(payload),
                                    "{\"temperature\":%s,\"pressure\":%s,"
                                    "\"humidity\":%s,\"co2\":%s,\"age\":%lu}",
                                    temperature, pressure, humidity, co2, u32_AgeSec);
   if ((payload_len < 0) || ((size_t)payload_len >= sizeof(payload)))
   {
      ESP_LOGW(TAG, "Sensor %u MQTT payload exceeds buffer capacity", (unsigned int)sensor_idx);
      return;
   }
   m_publish_message(topic, payload, payload_len, 0);
}

static void m_publish_output(const uint32_t channel)
{
   T_out_channel_data setup;
   int32_t value;
   char topic[MQTT_TOPIC_MAX_LEN];
   char payload[16];

   if ((output_get_setup(channel, &setup) != ESP_OK) ||
       ((setup.u8_OutType & OUTPUT_FLAG_REMOTE_RD) == 0u) ||
       (output_get_value(channel, &value) != ESP_OK))
   {
      return;
   }

   const int topic_len = snprintf(topic, sizeof(topic), "%s/outputs/%s/state", mpc_TopicPrefix, setup.ac_OutName);
   const int payload_len = snprintf(payload, sizeof(payload), "%ld", (long)value);
   if ((topic_len < 0) || ((size_t)topic_len >= sizeof(topic)) ||
       (payload_len < 0) || ((size_t)payload_len >= sizeof(payload)))
   {
      ESP_LOGW(TAG, "Output %lu MQTT state exceeds buffer capacity", (unsigned long)channel);
      return;
   }
   m_publish_message(topic, payload, payload_len, 0);
}

static void m_publish_state(void)
{
   const uint16_t sensor_count = sensor_get_count();
   for (uint16_t sensor_idx = 0u; sensor_idx < sensor_count; sensor_idx++)
   {
      m_publish_sensor(sensor_idx);
   }

   for (uint32_t channel = 0u; channel < OUTPUT_NUM_CHANNEL_MAX; channel++)
   {
      m_publish_output(channel);
   }
}

static void m_subscribe_output_commands(void)
{
   for (uint32_t channel = 0u; channel < OUTPUT_NUM_CHANNEL_MAX; channel++)
   {
      T_out_channel_data setup;
      char topic[MQTT_TOPIC_MAX_LEN];

      if ((output_get_setup(channel, &setup) == ESP_OK) &&
          ((setup.u8_OutType & OUTPUT_FLAG_REMOTE_WR) != 0u))
      {
         const int len = snprintf(topic, sizeof(topic), "%s/outputs/%s/set", mpc_TopicPrefix, setup.ac_OutName);
         if ((len < 0) || ((size_t)len >= sizeof(topic)))
         {
            ESP_LOGE(TAG, "Output command topic exceeds buffer capacity");
            continue;
         }
         if (esp_mqtt_client_subscribe(mqtt_client, topic, 1) < 0)
         {
            ESP_LOGW(TAG, "Failed to subscribe to %s", topic);
         }
      }
   }
}

static void m_handle_output_command(const char *topic, const size_t topic_len,
                                    const char *data, const size_t data_len)
{
   for (uint32_t channel = 0u; channel < OUTPUT_NUM_CHANNEL_MAX; channel++)
   {
      T_out_channel_data setup;
      char expected_topic[MQTT_TOPIC_MAX_LEN];
      char value_text[16];
      char *end;
      long value;
      const esp_err_t err = output_get_setup(channel, &setup);
      const int expected_len = snprintf(expected_topic, sizeof(expected_topic), "%s/outputs/%s/set", mpc_TopicPrefix, setup.ac_OutName);
      if ((err != ESP_OK) || (expected_len < 0) || ((size_t)expected_len >= sizeof(expected_topic)) ||
          ((size_t)expected_len != topic_len) ||
          (memcmp(topic, expected_topic, topic_len) != 0))
      {
         continue;
      }

      if ((setup.u8_OutType & OUTPUT_FLAG_REMOTE_WR) == 0u)
      {
         ESP_LOGW(TAG, "Rejected command for non-writable output %lu", (unsigned long)channel);
         return;
      }
      if (output_get_mode() != OUTPUT_MODE_AUTO)
      {
         ESP_LOGW(TAG, "Rejected command for output %lu: controller is not in AUTO mode",
                  (unsigned long)channel);
         return;
      }
      if ((data_len == 0u) || (data_len >= sizeof(value_text)))
      {
         ESP_LOGW(TAG, "Rejected malformed command for output %lu", (unsigned long)channel);
         return;
      }
      memcpy(value_text, data, data_len);
      value_text[data_len] = '\0';
      for (size_t idx = 0u; idx < data_len; idx++)
      {
         if ((value_text[idx] < '0') || (value_text[idx] > '9'))
         {
            ESP_LOGW(TAG, "Rejected malformed command for output %lu", (unsigned long)channel);
            return;
         }
      }
      errno = 0;
      value = strtol(value_text, &end, 10);
      if ((errno != 0) || (end == value_text) || (*end != '\0') ||
          (value < OUTPUT_PWM_MIN) || (value > OUTPUT_PWM_MAX))
      {
         ESP_LOGW(TAG, "Rejected out-of-range command for output %lu", (unsigned long)channel);
         return;
      }
      if (output_set_value(channel, (int32_t)value) != ESP_OK)
      {
         ESP_LOGE(TAG, "Failed to set output %lu", (unsigned long)channel);
      }
      return;
   }
}

static void m_mqtt_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
   (void)arg;
   (void)event_base;
   esp_mqtt_event_handle_t event = event_data;

   switch ((esp_mqtt_event_id_t)event_id)
   {
      case MQTT_EVENT_CONNECTED:
         mqtt_connected = true;
         ESP_LOGI(TAG, "Connected to broker");
         m_subscribe_output_commands();
         m_publish_state();
         m_publish_message(mac_StatusTopic, "online", 0, 1);
         io_set_led(STATUS_LED_WLAN, WIFI_STATUS_LED_MQTT, 0xFFFFFFFFuL);
         break;
      case MQTT_EVENT_DISCONNECTED:
         mqtt_connected = false;
         ESP_LOGW(TAG, "Disconnected from broker");
         io_set_led(STATUS_LED_WLAN, WIFI_STATUS_LED_CONNECT, 0xFFFFFFFFuL);
         break;
      case MQTT_EVENT_DATA:
         if ((event->current_data_offset == 0) &&
             (event->total_data_len == event->data_len) &&
             (event->topic_len > 0) && (event->data_len >= 0))
         {
            m_handle_output_command(event->topic, (size_t)event->topic_len,
                                    event->data, (size_t)event->data_len);
         }
         else
         {
            ESP_LOGW(TAG, "Ignoring fragmented or invalid MQTT command");
         }
         break;
      case MQTT_EVENT_ERROR:
         ESP_LOGW(TAG, "MQTT transport or broker error");
         break;
      default:
         break;
   }
}

static void m_mqtt_publish_task(void *arg)
{
   (void)arg;
   while (true)
   {
      if (mqtt_connected)
      {
         m_publish_state();
      }
      uint32_t interval = (mpt_MqttData != NULL)
                              ? mpt_MqttData->pt_MqttSetup->u32_PublishIntervalMs : CONFIG_CLIMATRONIC_MQTT_PUBLISH_INTERVAL_MS;
      vTaskDelay(pdMS_TO_TICKS(interval));
   }
}

esp_err_t mqtt_client_start(T_mqtt_data * const opt_MqttData)
{
   if (opt_MqttData == NULL)
   {
      return ESP_ERR_INVALID_ARG;
   }
   mpt_MqttData = opt_MqttData;
   if (opt_MqttData->ac_BrokerUri[0] == '\0')
   {
      mqtt_client_stop();
      ESP_LOGI(TAG, "MQTT disabled: broker URI is not configured");
      return ESP_OK;
   }
   if ((opt_MqttData->ac_TopicPrefix[0] == '\0') ||
       (strchr(opt_MqttData->ac_TopicPrefix, '+') != NULL) ||
       (strchr(opt_MqttData->ac_TopicPrefix, '#') != NULL))
   {
      mpc_TopicPrefix = NULL;
      ESP_LOGE(TAG, "MQTT topic prefix is empty or contains wildcard characters");
      return ESP_ERR_INVALID_ARG;
   }

   if (mqtt_client != NULL)
   {
      mqtt_client_stop();
   }
   mpc_TopicPrefix = opt_MqttData->ac_TopicPrefix;
   const int status_len = snprintf(mac_StatusTopic, sizeof(mac_StatusTopic), "%s/status", mpc_TopicPrefix);
   if ((status_len < 0) || ((size_t)status_len >= sizeof(mac_StatusTopic)))
   {
      ESP_LOGE(TAG, "Failed to create MQTT status topic");
      return ESP_FAIL;
   }

   const esp_mqtt_client_config_t config =
   {
      .broker =
      {
         .address.uri = opt_MqttData->ac_BrokerUri,
         .verification.crt_bundle_attach = esp_crt_bundle_attach,
      },
      .credentials =
      {
         .username = (opt_MqttData->ac_BrokerUserName[0] != '\0')
                         ? opt_MqttData->ac_BrokerUserName : NULL,
         .authentication.password = (opt_MqttData->ac_BrokerUserPw[0] != '\0')
                                        ? opt_MqttData->ac_BrokerUserPw : NULL,
      },
      .session =
      {
         .last_will =
         {
            .topic = mac_StatusTopic,
            .msg = "offline",
            .msg_len = 7,
            .qos = 1,
            .retain = 1,
         },
      },
   };

   mqtt_client = esp_mqtt_client_init(&config);
   if (mqtt_client == NULL)
   {
      ESP_LOGE(TAG, "Failed to create MQTT client");
      return ESP_FAIL;
   }

   esp_err_t err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                                   m_mqtt_event_handler, NULL);
   if (err == ESP_OK)
   {
      err = esp_mqtt_client_start(mqtt_client);
   }
   if (err != ESP_OK)
   {
      ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
      (void)esp_mqtt_client_destroy(mqtt_client);
      mqtt_client = NULL;
      return err;
   }

   if (mt_PublishTaskHandle == NULL &&
       xTaskCreate(m_mqtt_publish_task, "mqtt_publish", 4096u, NULL, 5u,
                   &mt_PublishTaskHandle) != pdPASS)
   {
      ESP_LOGE(TAG, "Failed to create MQTT publishing task");
      mqtt_client_stop();
      return ESP_ERR_NO_MEM;
   }
   return ESP_OK;
}

esp_err_t mqtt_client_restart(void)
{
   if (mpt_MqttData == NULL)
   {
      mqtt_data_get(&mt_MqttDataRuntime);
      mpt_MqttData = &mt_MqttDataRuntime;
   }
   mqtt_client_stop();

   return mqtt_client_start(mpt_MqttData);
}

bool mqtt_client_is_connected(void)
{
   return mqtt_connected;
}

void mqtt_client_stop(void)
{
   if (mqtt_client != NULL)
   {
      mqtt_connected = false;
      (void)esp_mqtt_client_stop(mqtt_client);
      (void)esp_mqtt_client_destroy(mqtt_client);
      mqtt_client = NULL;
   }
}