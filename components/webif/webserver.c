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
#include <stdbool.h>
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
static esp_err_t m_get_handler_wifi_config(httpd_req_t *req);
static esp_err_t m_set_handler_output(httpd_req_t *req);
static esp_err_t m_set_handler_sensor_setup(httpd_req_t *req);
static esp_err_t m_set_handler_output_setup(httpd_req_t *req);
static esp_err_t m_set_handler_system_setup(httpd_req_t *req);
static esp_err_t m_set_handler_wifi_config(httpd_req_t *req);
static esp_err_t m_post_handler_restart(httpd_req_t *req);
static esp_err_t m_set_handler_mode(httpd_req_t *req);
static size_t m_escape_wifi_ssid(const uint8_t *ssid, const size_t ssid_len, char *dst, const size_t dst_size);
static bool m_json_parse_string(const char **cursor, char *dst, const size_t dst_size);

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

static void m_json_skip_whitespace(const char **cursor)
{
   while ((**cursor == ' ') || (**cursor == '\t') || (**cursor == '\r') || (**cursor == '\n'))
   {
      (*cursor)++;
   }
}

static bool m_json_read_hex4(const char **cursor, uint32_t *value)
{
   uint32_t result = 0u;
   for (uint8_t i = 0u; i < 4u; i++)
   {
      const char c = *(*cursor)++;
      if (c == '\0')
      {
         return false;
      }
      uint8_t digit;
      if ((c >= '0') && (c <= '9'))
      {
         digit = (uint8_t)(c - '0');
      }
      else if ((c >= 'a') && (c <= 'f'))
      {
         digit = (uint8_t)(c - 'a' + 10);
      }
      else if ((c >= 'A') && (c <= 'F'))
      {
         digit = (uint8_t)(c - 'A' + 10);
      }
      else
      {
         return false;
      }
      result = (result << 4u) | digit;
   }
   *value = result;
   return true;
}

static bool m_json_append_codepoint(char *dst, const size_t dst_size, size_t *length, const uint32_t codepoint)
{
   uint8_t encoded[4];
   size_t encoded_length;

   if ((codepoint == 0u) || (codepoint > 0x10FFFFu) ||
       ((codepoint >= 0xD800u) && (codepoint <= 0xDFFFu)))
   {
      return false;
   }
   if (codepoint <= 0x7Fu)
   {
      encoded[0] = (uint8_t)codepoint;
      encoded_length = 1u;
   }
   else if (codepoint <= 0x7FFu)
   {
      encoded[0] = (uint8_t)(0xC0u | (codepoint >> 6u));
      encoded[1] = (uint8_t)(0x80u | (codepoint & 0x3Fu));
      encoded_length = 2u;
   }
   else if (codepoint <= 0xFFFFu)
   {
      encoded[0] = (uint8_t)(0xE0u | (codepoint >> 12u));
      encoded[1] = (uint8_t)(0x80u | ((codepoint >> 6u) & 0x3Fu));
      encoded[2] = (uint8_t)(0x80u | (codepoint & 0x3Fu));
      encoded_length = 3u;
   }
   else
   {
      encoded[0] = (uint8_t)(0xF0u | (codepoint >> 18u));
      encoded[1] = (uint8_t)(0x80u | ((codepoint >> 12u) & 0x3Fu));
      encoded[2] = (uint8_t)(0x80u | ((codepoint >> 6u) & 0x3Fu));
      encoded[3] = (uint8_t)(0x80u | (codepoint & 0x3Fu));
      encoded_length = 4u;
   }

   if ((*length + encoded_length) >= dst_size)
   {
      return false;
   }
   memcpy(&dst[*length], encoded, encoded_length);
   *length += encoded_length;
   return true;
}

static bool m_json_parse_string(const char **cursor, char *dst, const size_t dst_size)
{
   size_t length = 0u;
   if ((**cursor) != '"')
   {
      return false;
   }
   (*cursor)++;

   while (**cursor != '\0')
   {
      uint32_t codepoint;
      const uint8_t c = (uint8_t)*(*cursor)++;
      if (c == '"')
      {
         dst[length] = '\0';
         return true;
      }
      if (c == '\\')
      {
         const char escaped = *(*cursor)++;
         switch (escaped)
         {
            case '"': codepoint = '"'; break;
            case '\\': codepoint = '\\'; break;
            case '/': codepoint = '/'; break;
            case 'b': codepoint = '\b'; break;
            case 'f': codepoint = '\f'; break;
            case 'n': codepoint = '\n'; break;
            case 'r': codepoint = '\r'; break;
            case 't': codepoint = '\t'; break;
            case 'u':
            {
               uint32_t low;
               if (!m_json_read_hex4(cursor, &codepoint))
               {
                  return false;
               }
               if ((codepoint >= 0xD800u) && (codepoint <= 0xDBFFu))
               {
                  if ((**cursor != '\\') || ((*cursor)[1] != 'u'))
                  {
                     return false;
                  }
                  *cursor += 2;
                  if (!m_json_read_hex4(cursor, &low) || (low < 0xDC00u) || (low > 0xDFFFu))
                  {
                     return false;
                  }
                  codepoint = 0x10000u + (((codepoint - 0xD800u) << 10u) | (low - 0xDC00u));
               }
               else if ((codepoint >= 0xDC00u) && (codepoint <= 0xDFFFu))
               {
                  return false;
               }
               break;
            }
            default:
               return false;
         }
      }
      else
      {
         if (c < 0x20u)
         {
            return false;
         }
         if (c >= 0x80u)
         {
            if ((length + 1u) >= dst_size)
            {
               return false;
            }
            dst[length++] = (char)c;
            continue;
         }
         codepoint = c;
      }
      if (!m_json_append_codepoint(dst, dst_size, &length, codepoint))
      {
         return false;
      }
   }
   return false;
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

static esp_err_t m_get_handler_wifi_config(httpd_req_t *req)
{
   T_device_data device_data;
   char hostname[DEVICE_HOST_NAME_MAX * 6u + 1u];
   char ssid[WIFI_SSID_MAX * 6u + 1u];
   char bssid[18u] = "";
   char response[512u];
   int ret;

   device_data_get(&device_data);
   m_escape_wifi_ssid((const uint8_t *)device_data.ac_HostName,
                      strlen(device_data.ac_HostName),
                      hostname,
                      sizeof(hostname));
   m_escape_wifi_ssid((const uint8_t *)device_data.ac_WiFiSSID,
                      strlen(device_data.ac_WiFiSSID),
                      ssid,
                      sizeof(ssid));
   if (device_data_not_zero(device_data.au8_BSSID, WIFI_BSSID_SIZE))
   {
      ret = snprintf(bssid, sizeof(bssid), "%02X:%02X:%02X:%02X:%02X:%02X",
                     (unsigned int)device_data.au8_BSSID[0],
                     (unsigned int)device_data.au8_BSSID[1],
                     (unsigned int)device_data.au8_BSSID[2],
                     (unsigned int)device_data.au8_BSSID[3],
                     (unsigned int)device_data.au8_BSSID[4],
                     (unsigned int)device_data.au8_BSSID[5]);
      if ((ret <= 0) || ((size_t)ret >= sizeof(bssid)))
      {
         httpd_resp_send_500(req);
         return ESP_FAIL;
      }
   }

   ret = snprintf(response,
                  sizeof(response),
                  "{\"hostname\":\"%s\",\"ssid\":\"%s\",\"bssid\":\"%s\","
                  "\"passwordSet\":%s,\"openNetwork\":%s,"
                  "\"limits\":{\"hostname\":%u,\"ssid\":%u,\"password\":%u}}",
                  hostname,
                  ssid,
                  bssid,
                  (device_data.ac_WiFiPw[0] != '\0') ? "true" : "false",
                  (device_data.ac_WiFiPw[0] == '\0') ? "true" : "false",
                  (unsigned int)DEVICE_HOST_NAME_MAX,
                  (unsigned int)WIFI_SSID_MAX,
                  (unsigned int)WIFI_PASSWORD_MAX);
   if ((ret <= 0) || ((size_t)ret >= sizeof(response)))
   {
      httpd_resp_send_500(req);
      return ESP_FAIL;
   }

   httpd_resp_set_type(req, "application/json");
   httpd_resp_send(req, response, (size_t)ret);
   return ESP_OK;
}

static bool m_parse_wifi_config_json(const char *json,
                                     char *hostname,
                                     const size_t hostname_size,
                                     char *ssid,
                                     const size_t ssid_size,
                                     char *bssid,
                                     const size_t bssid_size,
                                     char *password,
                                     const size_t password_size,
                                     bool *open_network)
{
   const char *cursor = json;
   uint8_t fields = 0u;
   m_json_skip_whitespace(&cursor);
   if (*cursor++ != '{')
   {
      return false;
   }

   while (true)
   {
      char key[24u];
      m_json_skip_whitespace(&cursor);
      if (*cursor == '}')
      {
         cursor++;
         break;
      }
      if (!m_json_parse_string(&cursor, key, sizeof(key)))
      {
         return false;
      }
      m_json_skip_whitespace(&cursor);
      if (*cursor++ != ':')
      {
         return false;
      }
      m_json_skip_whitespace(&cursor);

      uint8_t field;
      if (strcmp(key, "hostname") == 0)
      {
         field = 0x01u;
         if (!m_json_parse_string(&cursor, hostname, hostname_size))
         {
            return false;
         }
      }
      else if (strcmp(key, "ssid") == 0)
      {
         field = 0x02u;
         if (!m_json_parse_string(&cursor, ssid, ssid_size))
         {
            return false;
         }
      }
      else if (strcmp(key, "bssid") == 0)
      {
         field = 0x04u;
         if (!m_json_parse_string(&cursor, bssid, bssid_size))
         {
            return false;
         }
      }
      else if (strcmp(key, "password") == 0)
      {
         field = 0x08u;
         if (!m_json_parse_string(&cursor, password, password_size))
         {
            return false;
         }
      }
      else if (strcmp(key, "openNetwork") == 0)
      {
         field = 0x10u;
         if (strncmp(cursor, "true", 4u) == 0)
         {
            *open_network = true;
            cursor += 4;
         }
         else if (strncmp(cursor, "false", 5u) == 0)
         {
            *open_network = false;
            cursor += 5;
         }
         else
         {
            return false;
         }
      }
      else
      {
         return false;
      }

      if ((fields & field) != 0u)
      {
         return false;
      }
      fields |= field;
      m_json_skip_whitespace(&cursor);
      if (*cursor == ',')
      {
         cursor++;
      }
      else if (*cursor != '}')
      {
         return false;
      }
   }

   m_json_skip_whitespace(&cursor);
   return (fields == 0x1Fu) && (*cursor == '\0');
}

static int8_t m_wifi_hex_value(const char c)
{
   if ((c >= '0') && (c <= '9'))
   {
      return (int8_t)(c - '0');
   }
   if ((c >= 'a') && (c <= 'f'))
   {
      return (int8_t)(c - 'a' + 10);
   }
   if ((c >= 'A') && (c <= 'F'))
   {
      return (int8_t)(c - 'A' + 10);
   }
   return -1;
}

static bool m_parse_wifi_bssid(const char *text, uint8_t *bssid)
{
   if (text[0] == '\0')
   {
      memset(bssid, 0, WIFI_BSSID_SIZE);
      return true;
   }
   if (strlen(text) != 17u)
   {
      return false;
   }
   for (uint8_t i = 0u; i < WIFI_BSSID_SIZE; i++)
   {
      const size_t offset = (size_t)i * 3u;
      const int8_t high = m_wifi_hex_value(text[offset]);
      const int8_t low = m_wifi_hex_value(text[offset + 1u]);
      if ((high < 0) || (low < 0) || ((i < WIFI_BSSID_SIZE - 1u) && (text[offset + 2u] != ':')))
      {
         return false;
      }
      bssid[i] = (uint8_t)(((uint8_t)high << 4u) | (uint8_t)low);
   }
   return true;
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

         output_get_value(u32_Channel, &s32_Pwm);
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
      output_set_value(channel, pwm * 10);
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

static esp_err_t m_set_handler_wifi_config(httpd_req_t *req)
{
   char body[1024u];
   char hostname[DEVICE_HOST_NAME_MAX + 1u] = "";
   char ssid[WIFI_SSID_MAX + 1u] = "";
   char bssid_text[18u] = "";
   char password[WIFI_PASSWORD_MAX + 1u] = "";
   uint8_t bssid[WIFI_BSSID_SIZE];
   bool open_network = false;
   size_t received = 0u;
   T_device_data current_data;

   if ((req->content_len == 0u) || (req->content_len >= sizeof(body)))
   {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request size");
      return ESP_ERR_INVALID_SIZE;
   }
   while (received < req->content_len)
   {
      const int ret = httpd_req_recv(req, &body[received], req->content_len - received);
      if (ret <= 0)
      {
         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete request body");
         return ESP_FAIL;
      }
      received += (size_t)ret;
   }
   body[received] = '\0';
   device_data_get(&current_data);

   if (!m_parse_wifi_config_json(body,
                                 hostname, sizeof(hostname),
                                 ssid, sizeof(ssid),
                                 bssid_text, sizeof(bssid_text),
                                 password, sizeof(password),
                                 &open_network) ||
       (strnlen(hostname, sizeof(hostname)) == 0u) ||
       (strnlen(hostname, sizeof(hostname)) > DEVICE_HOST_NAME_MAX) ||
       (strnlen(ssid, sizeof(ssid)) == 0u) ||
       (strnlen(ssid, sizeof(ssid)) > WIFI_SSID_MAX) ||
       (strnlen(password, sizeof(password)) > WIFI_PASSWORD_MAX) ||
       (open_network && (password[0] != '\0')) ||
       (!open_network && (password[0] == '\0') &&
        ((strcmp(ssid, current_data.ac_WiFiSSID) != 0) || (current_data.ac_WiFiPw[0] == '\0'))) ||
       !m_parse_wifi_bssid(bssid_text, bssid))
   {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid WiFi configuration");
      return ESP_ERR_INVALID_ARG;
   }

   esp_err_t err = device_data_set_hostname(hostname);
   if (err == ESP_OK)
   {
      err = device_data_set_ssid(ssid);
   }
   if (err == ESP_OK)
   {
      err = device_data_set_bssid(bssid);
   }
   if ((err == ESP_OK) && (open_network || (password[0] != '\0')))
   {
      err = device_data_set_wifipw(password);
   }
   if ((err == ESP_OK) && (device_data_store() != ESP_OK))
   {
      err = ESP_FAIL;
   }
   if (err != ESP_OK)
   {
      httpd_resp_send_500(req);
      return err;
   }

   if (httpd_resp_sendstr(req, "Saved; restarting") != ESP_OK)
   {
      return ESP_FAIL;
   }
   vTaskDelay(pdMS_TO_TICKS(500));
   system_sw_reset();
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
   config.max_uri_handlers = 16;
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
   httpd_uri_t get_wifi_config = {.uri="/api/wifi_config", .method=HTTP_GET, .handler=m_get_handler_wifi_config};
   httpd_uri_t set_output_uri = {.uri="/set_output", .method=HTTP_POST, .handler=m_set_handler_output, .user_ctx=NULL};
   httpd_uri_t set_sensor_setup_uri = {.uri="/set_sensor_setup", .method=HTTP_POST, .handler=m_set_handler_sensor_setup, .user_ctx=NULL};
   httpd_uri_t set_output_setup_uri = {.uri="/set_output_setup", .method=HTTP_POST, .handler=m_set_handler_output_setup, .user_ctx=NULL};
   httpd_uri_t set_system_setup_uri = {.uri="/set_system_setup", .method=HTTP_POST, .handler=m_set_handler_system_setup, .user_ctx=NULL};
   httpd_uri_t set_wifi_config_uri = {.uri="/set_wifi_config", .method=HTTP_POST, .handler=m_set_handler_wifi_config, .user_ctx=NULL};
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
   httpd_register_uri_handler(server, &get_wifi_config);
   httpd_register_uri_handler(server, &set_output_uri);
   httpd_register_uri_handler(server, &set_sensor_setup_uri);
   httpd_register_uri_handler(server, &set_output_setup_uri);
   httpd_register_uri_handler(server, &set_system_setup_uri);
   httpd_register_uri_handler(server, &set_wifi_config_uri);
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