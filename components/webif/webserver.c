/*
   webserver.c

   provides HTML web-interface
*/

/* --- Includes ------------------------------------------------------------ */
#include "webserver.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "sensor_api.h"
#include "output.h"
#include "system_data.h"
#include "device_data.h"
#include "system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>


/* --- Globals / Externals ------------------------------------------------- */
// HTM page index.html: externals for linker symbols
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* --- Defines ------------------------------------------------------------- */
#define OUTPUT_MODE_OFF       (0x00u)     // all outputs are off
#define OUTPUT_MODE_MANUAL    (0x01u)     // manual output control enabled
#define OUTPUT_MODE_AUTO      (0x02u)     // output controlled by software
#define WIFI_SCAN_RESULT_MAX  10u

/* --- Module globals ------------------------------------------------------ */
static httpd_handle_t server = NULL;
static uint8_t mu8_OutputMode = OUTPUT_MODE_AUTO;
static const char * mapc_OutputMode[3] = { "off", "manual", "auto" };

/* --- Types --------------------------------------------------------------- */

/* --- Prototypes ---------------------------------------------------------- */
static int32_t m_cStringValueFloat(char* const string, const int32_t size, char* const format, const float value);
static void m_escape_json_string(const char * src, char * const dst, const size_t size);
static esp_err_t m_get_handler_root(httpd_req_t *req);
static esp_err_t m_get_handler_favicon(httpd_req_t *req);
static esp_err_t m_get_handler_sensor_data(httpd_req_t *req);
static esp_err_t m_get_handler_output(httpd_req_t *req);
static esp_err_t m_get_handler_sensor_setup(httpd_req_t *req);
static esp_err_t m_get_handler_output_setup(httpd_req_t *req);
static esp_err_t m_get_handler_system_setup(httpd_req_t *req);
static esp_err_t m_get_handler_wifi_scan(httpd_req_t *req);
static esp_err_t m_set_handler_output(httpd_req_t *req);
static esp_err_t m_set_handler_sensor_setup(httpd_req_t *req);
static esp_err_t m_set_handler_output_setup(httpd_req_t *req);
static esp_err_t m_set_handler_system_setup(httpd_req_t *req);
static esp_err_t m_post_handler_restart(httpd_req_t *req);
static esp_err_t m_set_handler_mode(httpd_req_t *req);
static size_t m_escape_wifi_ssid(const uint8_t *ssid, const size_t ssid_len, char *dst, const size_t dst_size);

/* --- Implementation ------------------------------------------------------ */

static int32_t m_cStringValueFloat(char* const string, const int32_t size, char* const format, const float value)
{
   int32_t ret;

   if (value != SENSOR_VALUE_INVALID)
   {
      ret = snprintf(string, size, format, value);
   }
   else
   {
      ret = snprintf(string, size, "%s", "null");
   }

   return ret;
}

static void m_escape_json_string(const char * src, char * const dst, const size_t size)
{
   size_t idx = 0u;

   if ((src != NULL) && (dst != NULL) && (size > 0u))
   {
      while ((*src != '\0') && (idx + 1u < size))
      {
         const char c = *src++;

         if ((c == '\\') || (c == '"'))
         {
            if (idx + 2u >= size)
            {
               break;
            }
            dst[idx++] = '\\';
            dst[idx++] = c;
         }
         else if (c == '\n')
         {
            if (idx + 2u >= size)
            {
               break;
            }
            dst[idx++] = '\\';
            dst[idx++] = 'n';
         }
         else if (c == '\r')
         {
            if (idx + 2u >= size)
            {
               break;
            }
            dst[idx++] = '\\';
            dst[idx++] = 'r';
         }
         else if (c == '\t')
         {
            if (idx + 2u >= size)
            {
               break;
            }
            dst[idx++] = '\\';
            dst[idx++] = 't';
         }
         else
         {
            dst[idx++] = c;
         }
      }
      dst[idx] = '\0';
   }
   else if (dst != NULL)
   {
      dst[0] = '\0';
   }
}

static size_t m_escape_wifi_ssid(const uint8_t *ssid, const size_t ssid_len, char *dst, const size_t dst_size)
{
   static const char hex[] = "0123456789ABCDEF";
   size_t idx = 0u;

   for (size_t i = 0u; i < ssid_len; i++)
   {
      const uint8_t c = ssid[i];
      if ((c == '"') || (c == '\\'))
      {
         if (idx + 2u >= dst_size)
         {
            break;
         }
         dst[idx++] = '\\';
         dst[idx++] = (char)c;
      }
      else if ((c < 0x20u) || (c == 0x7Fu))
      {
         if (idx + 6u >= dst_size)
         {
            break;
         }
         dst[idx++] = '\\';
         dst[idx++] = 'u';
         dst[idx++] = '0';
         dst[idx++] = '0';
         dst[idx++] = hex[c >> 4u];
         dst[idx++] = hex[c & 0x0Fu];
      }
      else
      {
         if (idx + 1u >= dst_size)
         {
            break;
         }
         dst[idx++] = (char)c;
      }
   }

   if (dst_size > 0u)
   {
      dst[idx] = '\0';
   }
   return idx;
}

/**
 * @brief provide the HTML root file index.html
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 */
static esp_err_t m_get_handler_root(httpd_req_t *req)
{
   const uint32_t html_len = index_html_end - index_html_start;

   httpd_resp_set_type(req, "text/html");
   httpd_resp_send(req, (const char *)index_html_start, html_len);
   return ESP_OK;
}

/**
 * @brief provide the HTML favicon (not implemented)
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 */
static esp_err_t m_get_handler_favicon(httpd_req_t *req)
{
   httpd_resp_send_404(req);
   return ESP_OK;
}

/**
 * @brief get handler to fetch sensor data
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 */
static esp_err_t m_get_handler_sensor_data(httpd_req_t *req)
{
   // response example (size = 32 + n x (82+name[32]+type[14] = 128)):
   // {"sensor_cnt":"6", "sensor":[{"name":"Sensor Name1","type":"BME280","bus":0,"temp":5.1,"pres":6.1,"humi":5.1,"co2":5.0},]}
   char resp[32+128*16];
   const int32_t resp_size = sizeof(resp);
   const uint32_t u32_Count = sensor_get_count();

   int32_t ret = snprintf(resp, resp_size, "{\"sensor_cnt\":%lu,\"sensors\":[ ", u32_Count);
   if (ret > 0)
   {
      int32_t idx = ret;
      for (uint32_t u32_Sensor = 0; u32_Sensor < u32_Count; u32_Sensor++)
      {
         esp_err_t t_RetVal = ESP_OK;
         T_sensor_info t_SensorInfo;
         T_sensor_data t_SensorData;

         t_RetVal = (sensor_get_info(u32_Sensor, &t_SensorInfo) == ESP_OK) ? t_RetVal : ESP_FAIL;
         t_RetVal = (sensor_get_data(u32_Sensor, &t_SensorData) == ESP_OK) ? t_RetVal : ESP_FAIL;

         if (t_RetVal == ESP_OK)
         {
            char s_temp[8], s_pres[8], s_humi[8], s_co2[8];

            m_cStringValueFloat(s_temp, sizeof(s_temp), "%5.1f", t_SensorData.f32_Temperature);
            m_cStringValueFloat(s_pres, sizeof(s_pres), "%6.1f", t_SensorData.f32_Pressure);
            m_cStringValueFloat(s_humi, sizeof(s_humi), "%5.1f", t_SensorData.f32_Humidity);
            m_cStringValueFloat(s_co2, sizeof(s_co2), "%5.0f", t_SensorData.f32_CO2);

            ret = snprintf(&resp[idx], resp_size-idx, "{\"name\":\"%s\",\"type\":\"%s\",\"bus\":%u,\"temp\":%s,\"pres\":%s,\"humi\":%s,\"co2\":%s},",
                           t_SensorInfo.s_Name, t_SensorInfo.pt_Sensor->s_Type, t_SensorInfo.u8_IicBus, s_temp, s_pres, s_humi, s_co2);
         }
         if ((t_RetVal == ESP_OK) && ((ret > 0) && (ret < (resp_size-idx))))
         {
            idx += ret;
         }
         else
         {
            break;
         }
      }
      sprintf(&resp[idx-1], " ]}");
   }
   httpd_resp_set_type(req, "application/json");
   httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
   return ESP_OK;
}

static esp_err_t m_get_handler_sensor_setup(httpd_req_t *req)
{
   char resp[32u + 160u * 16u];
   int32_t idx = 0;
   int32_t ret;
   uint16_t u16_AddedSensors = 0u;
   const uint16_t u16_SensorCount = sensor_get_count();

   ret = snprintf(resp, sizeof(resp), "{\"sensors\":[");
   if (ret <= 0)
   {
      httpd_resp_send_500(req);
      return ESP_FAIL;
   }
   idx = ret;

   for (uint16_t u16_SensorIdx = 0u; u16_SensorIdx < u16_SensorCount; u16_SensorIdx++)
   {
      T_sensor_info t_Info;
      char ac_Name[SENSOR_NAME_LEN_MAX * 2u + 1u];

      if (sensor_get_info(u16_SensorIdx, &t_Info) != ESP_OK)
      {
         httpd_resp_send_500(req);
         return ESP_FAIL;
      }

      m_escape_json_string(t_Info.s_Name, ac_Name, sizeof(ac_Name));
      ret = snprintf(&resp[idx], sizeof(resp) - idx,
                     "%s{\"index\":%u,\"bus\":%u,\"type\":\"%s\",\"name\":\"%s\",\"rd\":%u}",
                     (u16_AddedSensors > 0u) ? "," : "",
                     (unsigned int)u16_SensorIdx,
                     (unsigned int)t_Info.u8_IicBus,
                     t_Info.pt_Sensor->s_Type,
                     ac_Name,
                     (unsigned int)((t_Info.u8_Flags & SENSOR_FLAGS_REMOTE_RD) != 0u));
      if ((ret <= 0) || ((size_t)ret >= (sizeof(resp) - (size_t)idx)))
      {
         httpd_resp_send_500(req);
         return ESP_FAIL;
      }

      idx += ret;
      u16_AddedSensors++;
   }

   ret = snprintf(&resp[idx], sizeof(resp) - (size_t)idx, "]}");
   if ((ret <= 0) || ((size_t)ret >= (sizeof(resp) - (size_t)idx)))
   {
      httpd_resp_send_500(req);
      return ESP_FAIL;
   }

   httpd_resp_set_type(req, "application/json");
   httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
   return ESP_OK;
}

static esp_err_t m_get_handler_system_setup(httpd_req_t *req)
{
   char resp[32u + 32u * I2C_BUS_COUNT];
   size_t idx = 0u;
   int ret = snprintf(resp, sizeof(resp), "{\"buses\":[");

   if (ret <= 0 || (size_t)ret >= sizeof(resp))
   {
      httpd_resp_send_500(req);
      return ESP_FAIL;
   }
   idx = (size_t)ret;

   for (uint32_t u32_Bus = 0u; u32_Bus < I2C_BUS_COUNT; u32_Bus++)
   {
      uint8_t u8_Frequency;
      if (system_data_get_i2c_freq(u32_Bus, &u8_Frequency) != ESP_OK)
      {
         httpd_resp_send_500(req);
         return ESP_FAIL;
      }

      ret = snprintf(&resp[idx], sizeof(resp) - idx,
                     "%s{\"bus\":%lu,\"frequency\":%u}",
                     (u32_Bus > 0u) ? "," : "",
                     (unsigned long)u32_Bus,
                     (unsigned int)u8_Frequency);
      if (ret <= 0 || (size_t)ret >= sizeof(resp) - idx)
      {
         httpd_resp_send_500(req);
         return ESP_FAIL;
      }
      idx += (size_t)ret;
   }

   ret = snprintf(&resp[idx], sizeof(resp) - idx, "]}");
   if (ret <= 0 || (size_t)ret >= sizeof(resp) - idx)
   {
      httpd_resp_send_500(req);
      return ESP_FAIL;
   }

   httpd_resp_set_type(req, "application/json");
   httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
   return ESP_OK;
}

static esp_err_t m_get_handler_wifi_scan(httpd_req_t *req)
{
   wifi_mode_t mode;
   esp_err_t err = esp_wifi_get_mode(&mode);
   if (err != ESP_OK)
   {
      httpd_resp_send_500(req);
      return err;
   }
   if ((mode != WIFI_MODE_STA) && (mode != WIFI_MODE_APSTA))
   {
      httpd_resp_set_status(req, "409 Conflict");
      httpd_resp_sendstr(req, "WiFi scan requires station or AP+station mode");
      return ESP_FAIL;
   }

   err = esp_wifi_scan_start(NULL, true);
   if (err != ESP_OK)
   {
      httpd_resp_send_500(req);
      return err;
   }

   uint16_t scan_count = 0u;
   err = esp_wifi_scan_get_ap_num(&scan_count);
   if (err != ESP_OK)
   {
      httpd_resp_send_500(req);
      return err;
   }
   uint16_t ap_count = (scan_count > WIFI_SCAN_RESULT_MAX) ? WIFI_SCAN_RESULT_MAX : scan_count;

   wifi_ap_record_t ap_records[WIFI_SCAN_RESULT_MAX];
   if (ap_count > 0u)
   {
      err = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
      if (err != ESP_OK)
      {
         httpd_resp_send_500(req);
         return err;
      }

      for (uint16_t i = 0u; i < ap_count; i++)
      {
         for (uint16_t j = (uint16_t)(i + 1u); j < ap_count; j++)
         {
            if (ap_records[j].rssi > ap_records[i].rssi)
            {
               const wifi_ap_record_t tmp = ap_records[i];
               ap_records[i] = ap_records[j];
               ap_records[j] = tmp;
            }
         }
      }
   }

   if (scan_count > ap_count)
   {
      err = esp_wifi_clear_ap_list();
      if (err != ESP_OK)
      {
         httpd_resp_send_500(req);
         return err;
      }
   }

   httpd_resp_set_type(req, "application/json");
   if (httpd_resp_sendstr_chunk(req, "{\"aps\":[") != ESP_OK)
   {
      return ESP_FAIL;
   }

   for (uint16_t i = 0u; i < ap_count; i++)
   {
      char escaped_ssid[WIFI_SSID_MAX * 6u + 1u];
      char item[320];
      const size_t ssid_len = strnlen((const char *)ap_records[i].ssid, WIFI_SSID_MAX);
      m_escape_wifi_ssid(ap_records[i].ssid, ssid_len, escaped_ssid, sizeof(escaped_ssid));
      const int ret = snprintf(item,
                               sizeof(item),
                               "%s{\"ssid\":\"%s\",\"bssid\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
                               "\"rssi\":%d,\"channel\":%u,\"auth\":%u}",
                               (i == 0u) ? "" : ",",
                               escaped_ssid,
                               (unsigned int)ap_records[i].bssid[0],
                               (unsigned int)ap_records[i].bssid[1],
                               (unsigned int)ap_records[i].bssid[2],
                               (unsigned int)ap_records[i].bssid[3],
                               (unsigned int)ap_records[i].bssid[4],
                               (unsigned int)ap_records[i].bssid[5],
                               ap_records[i].rssi,
                               (unsigned int)ap_records[i].primary,
                               (unsigned int)ap_records[i].authmode);
      if ((ret <= 0) || ((size_t)ret >= sizeof(item)) ||
          (httpd_resp_sendstr_chunk(req, item) != ESP_OK))
      {
         return ESP_FAIL;
      }
   }

   if ((httpd_resp_sendstr_chunk(req, "]}") != ESP_OK) ||
       (httpd_resp_sendstr_chunk(req, "") != ESP_OK))
   {
      return ESP_FAIL;
   }
   return ESP_OK;
}

/**
 * @brief get handler to fetch output data
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 */
static esp_err_t m_get_handler_output(httpd_req_t *req)
{
   char resp[160];
   int32_t idx = 0;
   int32_t ret;

   // response example:
   // {"mode":"manual", "channels":[1000,1000,1000,1000,1000,1000,1000,1000]}
   resp[idx++] = '{';
   ret = sprintf(&resp[idx], "\"mode\":\"%s\", \"channels\":[", mapc_OutputMode[mu8_OutputMode]);
   if (ret > 0)
   {
      idx += ret;
      for (uint32_t u32_Channel = 0; u32_Channel < OUTPUT_NUM_CHANNEL_MAX; u32_Channel++)
      {
         int32_t s32_Pwm;

         output_get_pwm(u32_Channel, &s32_Pwm);
         s32_Pwm = (s32_Pwm + 5)/10;
         ret = snprintf(&resp[idx], (sizeof(resp)-idx), "%ld,", s32_Pwm);
         if (ret > 0)
         {
            idx += ret;
         }
         else
         {
            break;
         }
      }
   }
   sprintf(&resp[idx-1], "]}");

   httpd_resp_set_type(req, "application/json");
   httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
   return ESP_OK;
}

static esp_err_t m_get_handler_output_setup(httpd_req_t *req)
{
   char resp[768];
   int32_t idx = 0;
   int32_t ret;

   ret = snprintf(resp, sizeof(resp), "{\"channels\":[");
   if (ret > 0)
   {
      idx = ret;
      for (uint32_t u32_Channel = 0; u32_Channel < OUTPUT_NUM_CHANNEL_MAX; u32_Channel++)
      {
         T_out_channel_data t_Setup;
         char ac_Name[OUTPUT_NAME_LEN_MAX + 1u];

         if (output_get_setup(u32_Channel, &t_Setup) == ESP_OK)
         {
            m_escape_json_string(t_Setup.ac_OutName, ac_Name, sizeof(ac_Name));
            ret = snprintf(&resp[idx], sizeof(resp)-idx,
                           "%s{\"channel\":%lu,\"name\":\"%s\",\"type\":%u,\"value\":%u,\"rd\":%u,\"wr\":%u}",
                           (u32_Channel > 0u) ? "," : "",
                           (unsigned long)u32_Channel,
                           ac_Name,
                           (unsigned int)(t_Setup.u8_OutType & OUTPUT_TYPE_MASK),
                           (unsigned int)t_Setup.u16_OutValueDef,
                           (unsigned int)((t_Setup.u8_OutType & OUTPUT_FLAG_REMOTE_RD) != 0u),
                           (unsigned int)((t_Setup.u8_OutType & OUTPUT_FLAG_REMOTE_WR) != 0u));
            if ((ret > 0) && (ret < (sizeof(resp)-idx)))
            {
               idx += ret;
            }
            else
            {
               break;
            }
         }
      }
      ret = snprintf(&resp[idx], sizeof(resp)-idx, "]}");
      if ((ret > 0) && (ret < (sizeof(resp)-idx)))
      {
         idx += ret;
      }
   }

   httpd_resp_set_type(req, "application/json");
   httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
   return ESP_OK;
}

/**
 * @brief set handler to set output pwm values
 *
 * @return esp_err_t
 *  - ESP_OK:     Success
 */
static esp_err_t m_set_handler_output(httpd_req_t *req)
{
   char buf[100];
   int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
   if (ret <= 0) return ESP_FAIL;
   buf[ret] = '\0';
   int channel = 0;
   int pwm = 0;
   ret = sscanf(buf, "{\"channel\":%d,\"value\":%d}", &channel, &pwm); // JSON-Parser

   if ((ret == 2) && (channel >= 0) && (channel <= OUTPUT_NUM_CHANNEL_MAX))
   {
      output_set_pwm(channel, pwm * 10);
   }

   httpd_resp_sendstr(req, "OK");
   return ESP_OK;
}

static esp_err_t m_set_handler_output_setup(httpd_req_t *req)
{
   char buf[256];
   int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
   if (ret <= 0) return ESP_FAIL;
   buf[ret] = '\0';

   int channel = -1;
   int type = OUTPUT_TYPE_INVALID;
   int value = 0;
   int rd = 0;
   int wr = 0;
   char name[OUTPUT_NAME_LEN_MAX + 1u];

   ret = sscanf(buf, "{\"channel\":%d,\"name\":\"%24[^\"]\",\"type\":%d,\"value\":%d,\"rd\":%d,\"wr\":%d}",
                &channel, name, &type, &value, &rd, &wr);
   if ((ret == 6) && (channel >= 0) && (channel < OUTPUT_NUM_CHANNEL_MAX) &&
       (type >= OUTPUT_TYPE_DIGITAL) && (type <= OUTPUT_TYPE_PWM_AC) &&
       (value >= OUTPUT_PWM_MIN) && (value <= OUTPUT_PWM_MAX) &&
       (rd >= 0) && (rd <= 1) && (wr >= 0) && (wr <= 1))
   {
      T_out_channel_data t_Setup;
      const uint8_t u8_FlagRd = (rd != 0) ? OUTPUT_FLAG_REMOTE_RD : 0u;
      const uint8_t u8_FlagWr = (wr != 0) ? OUTPUT_FLAG_REMOTE_WR : 0u;
      esp_err_t err;

      strncpy(t_Setup.ac_OutName, name, OUTPUT_NAME_LEN_MAX);
      t_Setup.ac_OutName[OUTPUT_NAME_LEN_MAX] = '\0';
      t_Setup.u8_OutType = (uint8_t)type | u8_FlagRd | u8_FlagWr;
      t_Setup.u16_OutValueDef = (uint16_t)value;

      err = output_set_setup((uint32_t)channel, &t_Setup);
      if (err != ESP_OK)
      {
         httpd_resp_send_500(req);
         return err;
      }
   }
   else
   {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, NULL);
      return ESP_ERR_INVALID_SIZE;
   }

   httpd_resp_sendstr(req, "OK");
   return ESP_OK;
}

static esp_err_t m_set_handler_sensor_setup(httpd_req_t *req)
{
   char buf[128];
   int ret = httpd_req_recv(req, buf, sizeof(buf) - 1u);
   if (ret <= 0)
   {
      return ESP_FAIL;
   }
   buf[ret] = '\0';

   int sensor = -1;
   int rd = -1;
   char name[SENSOR_NAME_LEN_MAX + 1u];
   ret = sscanf(buf, "{\"sensor\":%d,\"name\":\"%24[^\"]\",\"rd\":%d}", &sensor, name, &rd);
   if ((ret != 3) || (sensor < 0) || (sensor >= sensor_get_count()) || ((rd != 0) && (rd != 1)))
   {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, NULL);
      return ESP_ERR_INVALID_ARG;
   }

   const uint8_t u8_Flags = (rd != 0) ? SENSOR_FLAGS_REMOTE_RD : 0u;
   const esp_err_t err = sensor_set_setup((uint16_t)sensor, name, u8_Flags);
   if (err != ESP_OK)
   {
      httpd_resp_send_500(req);
      return err;
   }

   httpd_resp_sendstr(req, "OK");
   return ESP_OK;
}

static esp_err_t m_set_handler_system_setup(httpd_req_t *req)
{
   char buf[96];
   int ret = httpd_req_recv(req, buf, sizeof(buf) - 1u);
   if (ret <= 0)
   {
      return ESP_FAIL;
   }
   buf[ret] = '\0';

   uint8_t au8_Frequency[I2C_BUS_COUNT];
   char *pc_Frequency = strstr(buf, "\"frequencies\":[");
   if (pc_Frequency == NULL)
   {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, NULL);
      return ESP_ERR_INVALID_ARG;
   }
   pc_Frequency += strlen("\"frequencies\":[");

   for (uint32_t u32_Bus = 0u; u32_Bus < I2C_BUS_COUNT; u32_Bus++)
   {
      char *pc_End;
      long s32_Frequency = strtol(pc_Frequency, &pc_End, 10);
      if ((pc_End == pc_Frequency) ||
          ((s32_Frequency != I2C_BUS_FREQ_100K) && (s32_Frequency != I2C_BUS_FREQ_400K)))
      {
         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, NULL);
         return ESP_ERR_INVALID_ARG;
      }
      au8_Frequency[u32_Bus] = (uint8_t)s32_Frequency;
      pc_Frequency = pc_End;
      if ((u32_Bus + 1u < I2C_BUS_COUNT) && (*pc_Frequency++ != ','))
      {
         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, NULL);
         return ESP_ERR_INVALID_ARG;
      }
   }
   if (strncmp(pc_Frequency, "]}", 2u) != 0)
   {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, NULL);
      return ESP_ERR_INVALID_ARG;
   }

   for (uint32_t u32_Bus = 0u; u32_Bus < I2C_BUS_COUNT; u32_Bus++)
   {
      const esp_err_t err = system_data_set_i2c_freq(u32_Bus, (uint8_t)au8_Frequency[u32_Bus]);
      if (err != ESP_OK)
      {
         httpd_resp_send_500(req);
         return err;
      }
   }

   const esp_err_t err = system_data_store();
   if (err != ESP_OK)
   {
      httpd_resp_send_500(req);
      return err;
   }

   httpd_resp_sendstr(req, "OK");
   return ESP_OK;
}

static esp_err_t m_post_handler_restart(httpd_req_t *req)
{
   httpd_resp_sendstr(req, "Restarting");
   vTaskDelay(pdMS_TO_TICKS(250));
   system_sw_reset();
   return ESP_OK;
}

/**
 * @brief set handler to set output mode
 *
 * @return esp_err_t
 *  - ESP_OK:     Success
 *  - ESP_FAIL:   Something went wrong with request data
 */
static esp_err_t m_set_handler_mode(httpd_req_t *req)
{
   char buf[100];
   int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
   if (ret <= 0) return ESP_FAIL;
   buf[ret] = '\0';
   if (strstr(buf, "manual"))
   {
      mu8_OutputMode = OUTPUT_MODE_MANUAL;
   }
   else if (strstr(buf, "auto"))
   {
      mu8_OutputMode = OUTPUT_MODE_AUTO;
   }
   else
   {
      mu8_OutputMode = OUTPUT_MODE_OFF;
   }
   httpd_resp_sendstr(req, "OK");
   return ESP_OK;
}

void start_webserver(void)
{
   httpd_config_t config = HTTPD_DEFAULT_CONFIG();
   config.max_uri_handlers = 14;
   config.stack_size = 8192;

   httpd_start(&server, &config);

   httpd_uri_t root = {.uri="/", .method=HTTP_GET, .handler=m_get_handler_root};
   httpd_uri_t favicon = {.uri="/favicon.ico", .method=HTTP_GET, .handler=m_get_handler_favicon};
   httpd_uri_t get_sensor_data = {.uri="/api/sensors", .method=HTTP_GET, .handler=m_get_handler_sensor_data};
   httpd_uri_t get_output_data = {.uri="/api/output", .method=HTTP_GET, .handler=m_get_handler_output};
   httpd_uri_t get_sensor_setup_data = {.uri="/api/sensor_setup", .method=HTTP_GET, .handler=m_get_handler_sensor_setup};
   httpd_uri_t get_output_setup_data = {.uri="/api/output_setup", .method=HTTP_GET, .handler=m_get_handler_output_setup};
   httpd_uri_t get_system_setup_data = {.uri="/api/system_setup", .method=HTTP_GET, .handler=m_get_handler_system_setup};
   httpd_uri_t get_wifi_scan = {.uri="/api/wifi_scan", .method=HTTP_GET, .handler=m_get_handler_wifi_scan};
   httpd_uri_t set_output_uri = {.uri="/set_output", .method=HTTP_POST, .handler=m_set_handler_output, .user_ctx=NULL};
   httpd_uri_t set_sensor_setup_uri = {.uri="/set_sensor_setup", .method=HTTP_POST, .handler=m_set_handler_sensor_setup, .user_ctx=NULL};
   httpd_uri_t set_output_setup_uri = {.uri="/set_output_setup", .method=HTTP_POST, .handler=m_set_handler_output_setup, .user_ctx=NULL};
   httpd_uri_t set_system_setup_uri = {.uri="/set_system_setup", .method=HTTP_POST, .handler=m_set_handler_system_setup, .user_ctx=NULL};
   httpd_uri_t restart_uri = {.uri="/restart", .method=HTTP_POST, .handler=m_post_handler_restart, .user_ctx=NULL};
   httpd_uri_t set_mode_uri = {.uri="/set_mode", .method=HTTP_POST, .handler=m_set_handler_mode, .user_ctx=NULL};

   httpd_register_uri_handler(server, &root);
   httpd_register_uri_handler(server, &favicon);
   httpd_register_uri_handler(server, &get_sensor_data);
   httpd_register_uri_handler(server, &get_sensor_setup_data);
   httpd_register_uri_handler(server, &get_output_data);
   httpd_register_uri_handler(server, &get_output_setup_data);
   httpd_register_uri_handler(server, &get_system_setup_data);
   httpd_register_uri_handler(server, &get_wifi_scan);
   httpd_register_uri_handler(server, &set_output_uri);
   httpd_register_uri_handler(server, &set_sensor_setup_uri);
   httpd_register_uri_handler(server, &set_output_setup_uri);
   httpd_register_uri_handler(server, &set_system_setup_uri);
   httpd_register_uri_handler(server, &restart_uri);
   httpd_register_uri_handler(server, &set_mode_uri);
}

void stop_webserver(void)
{
   if (server != NULL)
   {
      httpd_stop(server);
   }
}