/*
   webserver.c

   provides HTML web-interface
*/

/* --- Includes ------------------------------------------------------------ */
#include "webserver.h"
#include "esp_http_server.h"
#include "sensor.h"
#include "output.h"
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

/* --- Module globals ------------------------------------------------------ */
static uint8_t mu8_OutputMode = OUTPUT_MODE_AUTO;
static const char * mapc_OutputMode[3] = { "off", "manual", "auto" };

/* --- Types --------------------------------------------------------------- */

/* --- Prototypes ---------------------------------------------------------- */
static int32_t m_cStringValueFloat(char* const string, const int32_t size, char* const format, const float value);
static esp_err_t m_get_handler_root(httpd_req_t *req);
static esp_err_t m_get_handler_favicon(httpd_req_t *req);
static esp_err_t m_get_handler_sensor_data(httpd_req_t *req);
static esp_err_t m_get_handler_output(httpd_req_t *req);
static esp_err_t m_set_handler_output(httpd_req_t *req);
static esp_err_t m_set_handler_mode(httpd_req_t *req);

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
   char resp[32+128*8];
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
   httpd_handle_t server = NULL;

   httpd_start(&server, &config);

   httpd_uri_t root = {.uri="/", .method=HTTP_GET, .handler=m_get_handler_root};
   httpd_uri_t favicon = {.uri="/favicon.ico", .method=HTTP_GET, .handler=m_get_handler_favicon};
   httpd_uri_t get_sensor_data = {.uri="/api/sensors", .method=HTTP_GET, .handler=m_get_handler_sensor_data};
   httpd_uri_t get_output_data = {.uri="/api/output", .method=HTTP_GET, .handler=m_get_handler_output};
   httpd_uri_t set_output_uri = {.uri="/set_output", .method=HTTP_POST, .handler=m_set_handler_output, .user_ctx=NULL};
   httpd_uri_t set_mode_uri = {.uri="/set_mode", .method=HTTP_POST, .handler=m_set_handler_mode, .user_ctx=NULL};

   httpd_register_uri_handler(server, &root);
   httpd_register_uri_handler(server, &favicon);
   httpd_register_uri_handler(server, &get_sensor_data);
   httpd_register_uri_handler(server, &get_output_data);
   httpd_register_uri_handler(server, &set_output_uri);
   httpd_register_uri_handler(server, &set_mode_uri);
}
