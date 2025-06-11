/*
   webserver.c

   provides HTML web-interface
*/

/* --- Includes ------------------------------------------------------------ */
#include "webserver.h"
#include "esp_http_server.h"
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

/* --- Implementation ------------------------------------------------------ */

/**
 * @brief random noise generator to simulate sensor data
 *
 * @return float range [0.8f ... 1.2f]
 */
static float get_noise(void)
{
   // create random number in range -1.0 ...  +1.0
   float f_noise = (float)rand() * (2.0f / (float)RAND_MAX) - 1.0f;

   return f_noise * 0.2f + 1.0f;
}

/**
 * @brief provide the HTML root file index.html
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
   const uint32_t html_len = index_html_end - index_html_start;

   httpd_resp_set_type(req, "text/html");
   httpd_resp_send(req, (const char *)index_html_start, html_len);
   return ESP_OK;
}

/**
 * @brief get handler to fetch sensor data
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 */
static esp_err_t get_sensor_handler(httpd_req_t *req)
{
    char resp[100];
    float temp = 20.0f * get_noise();
    float hum = 50.0f * get_noise();
    float press = 850.0f * get_noise();

    snprintf(resp, sizeof(resp), "{\"temp\":%.1f,\"hum\":%.1f,\"press\":%.0f}",
             temp, hum, press );
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
static esp_err_t get_output_handler(httpd_req_t *req)
{
   char resp[160];
   int32_t idx = 0;
   int32_t ret;

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
esp_err_t set_output_handler(httpd_req_t *req)
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
esp_err_t set_mode_handler(httpd_req_t *req)
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

static esp_err_t favicon_get_handler(httpd_req_t *req)
{
   httpd_resp_send_404(req);
   return ESP_OK;
}

void start_webserver(void)
{
   httpd_config_t config = HTTPD_DEFAULT_CONFIG();
   httpd_handle_t server = NULL;

   srand(time(NULL));
   httpd_start(&server, &config);

   httpd_uri_t root = {.uri="/", .method=HTTP_GET, .handler=root_get_handler};
   httpd_uri_t favicon = {.uri="/favicon.ico", .method=HTTP_GET, .handler=favicon_get_handler};
   httpd_uri_t get_sensor_data = {.uri="/api/sensor", .method=HTTP_GET, .handler=get_sensor_handler};
   httpd_uri_t get_output_data = {.uri="/api/output", .method=HTTP_GET, .handler=get_output_handler};
   httpd_uri_t set_output_uri = {.uri="/set_output", .method=HTTP_POST, .handler=set_output_handler, .user_ctx=NULL};
   httpd_uri_t set_mode_uri = {.uri="/set_mode", .method=HTTP_POST, .handler=set_mode_handler, .user_ctx=NULL};

   httpd_register_uri_handler(server, &root);
   httpd_register_uri_handler(server, &favicon);
   httpd_register_uri_handler(server, &get_sensor_data);
   httpd_register_uri_handler(server, &get_output_data);
   httpd_register_uri_handler(server, &set_output_uri);
   httpd_register_uri_handler(server, &set_mode_uri);
}
