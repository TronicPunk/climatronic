/*
   wifi_connect.c
   establish WIFI connection
*/

/* --- Includes ------------------------------------------------------------ */
#include "wifi_connect.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include "mdns.h"
#include "inout.h"
#include <stdbool.h>
#include <string.h>

/* -- Defines ------------------------------------------------------------------------------------------------------- */

#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_RECONNECT_TIME_US      5000000ULL     // reconnect retry time delay in micro seconds
#define WIFI_CONNECT_TIMEOUT_MS     10000u         // WIFI connect timeout, start fallback Setup-AP after timeout
#define WIFI_STATUS_LED_NOWIFI      0x00C80000uL   // no WIFI: LED red
#define WIFI_STATUS_LED_NOACCESS    0x00C82000uL   // WIFI no access data: LED orange
#define WIFI_STATUS_LED_NOCONNECT   0x00A06000uL   // WIFI not connected: LED yellow
#define WIFI_STATUS_LED_CONNECT     0x0028F028uL   // WIFI connected: LED light green


/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */
static void m_wifi_retry_timer_cb(void *arg);
static void m_wifi_start_retry_timer(void);
static void m_wifi_stop_retry_timer(void);
static void m_wifi_log_connection_health(void);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */
static EventGroupHandle_t wifi_event_group;
static const char *TAG = "wifi";
static uint32_t wifi_disconnect_count = 0u;
static uint32_t wifi_connect_count = 0u;
static int8_t wifi_last_rssi = 0;
static esp_timer_handle_t wifi_retry_timer = NULL;
static bool wifi_station_enabled = false;

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * cyclic wifi connect retry timer, started when WIFI connection is lost
 */
static void m_wifi_retry_timer_cb(void *arg)
{
   if (wifi_station_enabled && (wifi_event_group != NULL) &&
       ((xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) == 0u))
   {
//       ESP_LOGW(TAG, "Retrying WiFi connection after disconnect...");
       esp_wifi_connect();
   }
}

/**
 * start cyclic wifi re-connect timer
 */
static void m_wifi_start_retry_timer(void)
{
   if (wifi_retry_timer != NULL)
   {
      esp_timer_start_periodic(wifi_retry_timer, WIFI_RECONNECT_TIME_US);
   }
}

/**
 * stop cyclic wifi re-connect timer
 */
static void m_wifi_stop_retry_timer(void)
{
   if (wifi_retry_timer != NULL)
   {
       esp_timer_stop(wifi_retry_timer);
   }
}

/**
 * get wifi connection status
 */
static void m_wifi_log_connection_health(void)
{
   wifi_ap_record_t ap_info;
   memset(&ap_info, 0, sizeof(ap_info));

   if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
   {
       wifi_last_rssi = ap_info.rssi;
       ESP_LOGI(TAG,
                "WiFi health: connect #%lu, SSID=%s, RSSI=%d dBm, ch=%d, auth=%d",
                (unsigned long)wifi_connect_count,
                ap_info.ssid,
                ap_info.rssi,
                ap_info.primary,
                ap_info.authmode);
   }
   else
   {
      ESP_LOGI(TAG, "WiFi health: connect #%lu, no AP info available", (unsigned long)wifi_connect_count);
   }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
   if (event_base == WIFI_EVENT)
   {
      switch (event_id)
      {
         case WIFI_EVENT_STA_START:
         {
            ESP_LOGI(TAG, "WiFi STA started");
            if (wifi_station_enabled)
            {
               esp_wifi_connect();
            }
            break;
         }

         case WIFI_EVENT_STA_DISCONNECTED:
         {
            if (!wifi_station_enabled)
            {
               break;
            }
            wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
            io_set_led(STATUS_LED_WLAN, WIFI_STATUS_LED_NOCONNECT);
            wifi_disconnect_count++;
            ESP_LOGW(TAG,
                     "WiFi disconnected #%lu, reason=%d, last_rssi=%d dBm, reconnecting...",
                     (unsigned long)wifi_disconnect_count,
                     event ? event->reason : -1,
                     wifi_last_rssi);
            m_wifi_start_retry_timer();
            break;
         }

         case WIFI_EVENT_AP_START:
         {
            ESP_LOGI(TAG, "Setup access point started; open http://192.168.4.1");
            break;
         }

         default:
            break;
      }
   }
   else if ((event_base == IP_EVENT) && (event_id == IP_EVENT_STA_GOT_IP) && wifi_station_enabled)
   {
      m_wifi_stop_retry_timer();
      xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
      wifi_connect_count++;
      m_wifi_log_connection_health();
      io_set_led(STATUS_LED_WLAN, WIFI_STATUS_LED_CONNECT);
      ESP_LOGI(TAG, "Connected to WiFi -- total connects: %lu", (unsigned long)wifi_connect_count);
   }
}

static void m_wifi_start_setup_ap(const bool wifi_already_started)
{
   wifi_config_t ap_config = { 0 };
   const size_t ssid_len = strlen(CONFIG_SETUP_AP_SSID);
   const size_t password_len = strlen(CONFIG_SETUP_AP_PASSWORD);

   ESP_ERROR_CHECK((ssid_len > 0u && ssid_len <= sizeof(ap_config.ap.ssid) &&
                    password_len >= 8u && password_len < sizeof(ap_config.ap.password))
                       ? ESP_OK
                       : ESP_ERR_INVALID_ARG);

   wifi_station_enabled = false;
   m_wifi_stop_retry_timer();
   ESP_ERROR_CHECK(esp_netif_create_default_wifi_ap() == NULL ? ESP_FAIL : ESP_OK);

   memcpy(ap_config.ap.ssid, CONFIG_SETUP_AP_SSID, ssid_len);
   ap_config.ap.ssid_len = (uint8_t)ssid_len;
   memcpy(ap_config.ap.password, CONFIG_SETUP_AP_PASSWORD, password_len);
   ap_config.ap.channel = 1u;
   ap_config.ap.max_connection = 4;
   ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

   ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
   ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
   if (!wifi_already_started)
   {
      ESP_ERROR_CHECK(esp_wifi_start());
   }
   ESP_LOGI(TAG, "Setup AP SSID=%s; web interface at http://192.168.4.1", CONFIG_SETUP_AP_SSID);
   io_set_led(STATUS_LED_WLAN, WIFI_STATUS_LED_NOACCESS);
}

void wifi_connect(T_device_data * const opt_DeviceData)
{
   // create WIFI event group
   wifi_event_group = xEventGroupCreate();
   ESP_ERROR_CHECK((wifi_event_group == NULL) ? ESP_FAIL : ESP_OK);
   xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

   // create WIFI reconnect timer
   const esp_timer_create_args_t timer_args =
   {
      .callback = &m_wifi_retry_timer_cb,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "wifi_retry"
   };
   ESP_ERROR_CHECK(esp_timer_create(&timer_args, &wifi_retry_timer));

   ESP_ERROR_CHECK(esp_netif_init());
   ESP_ERROR_CHECK(esp_event_loop_create_default());
   ESP_ERROR_CHECK(esp_netif_create_default_wifi_sta() == NULL ? ESP_FAIL : ESP_OK);

   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   ESP_ERROR_CHECK(esp_wifi_init(&cfg));

   esp_event_handler_instance_t instance_any_id, instance_got_ip;
   ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                       ESP_EVENT_ANY_ID,
                                                       &wifi_event_handler,
                                                       NULL,
                                                       &instance_any_id));
   ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                       IP_EVENT_STA_GOT_IP,
                                                       &wifi_event_handler,
                                                       NULL,
                                                       &instance_got_ip));
   wifi_config_t wifi_config = { 0 };
   const size_t ssid_len = strnlen(opt_DeviceData->ac_WiFiSSID, sizeof(wifi_config.sta.ssid));
   if (ssid_len == 0u)
   {
      ESP_LOGW(TAG, "No WiFi SSID configured; starting setup access point");
      m_wifi_start_setup_ap(false);
      return;
   }
   memcpy(wifi_config.sta.ssid, opt_DeviceData->ac_WiFiSSID, ssid_len);
   strncpy((char *)wifi_config.sta.password, opt_DeviceData->ac_WiFiPw, sizeof(wifi_config.sta.password));
   wifi_config.sta.password[sizeof(wifi_config.sta.password)-1u] = 0u;
   if (device_data_not_zero(opt_DeviceData->au8_BSSID, WIFI_BSSID_SIZE))
   {
      wifi_config.sta.bssid_set = true;
      memcpy(wifi_config.sta.bssid, opt_DeviceData->au8_BSSID, WIFI_BSSID_SIZE);
   }

   wifi_station_enabled = true;
   ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
   ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
   ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
   ESP_ERROR_CHECK(esp_wifi_start());
   xEventGroupWaitBits(wifi_event_group,
                       WIFI_CONNECTED_BIT,
                       false,
                       true,
                       pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
   if ((xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) == 0u)
   {
      ESP_LOGW(TAG, "WiFi connection failed; starting setup access point");
      m_wifi_start_setup_ap(true);
   }
}

void wifi_reset(void)
{
   wifi_station_enabled = false;
   m_wifi_stop_retry_timer();
   esp_wifi_disconnect();
   esp_wifi_stop();
   esp_wifi_deinit();
   if (wifi_retry_timer != NULL)
   {
       esp_timer_delete(wifi_retry_timer);
       wifi_retry_timer = NULL;
   }
   vEventGroupDelete(wifi_event_group);
   wifi_event_group = NULL;
}

void start_mdns_service(const char* const oac_HostName)
{
    mdns_init();
    mdns_hostname_set(oac_HostName);      // Hostname: <host_name>.local
    mdns_instance_name_set("ESP32 Web");

    // Optional: Dienst registrieren
    mdns_service_add("WebServer", "_http", "_tcp", 80, NULL, 0);
}

void stop_mdns_service(void)
{
   mdns_free();
}