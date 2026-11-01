// === wifi_connect.c ===
#include "wifi_connect.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include "mdns.h"
#include <string.h>


#define WIFI_SSID       "<tbd>"
#define WIFI_PASS       "<tbd>"

#define ESP32_HOST_NAME "clima_control"
#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t wifi_event_group;
static const char *TAG = "wifi";
static uint32_t wifi_disconnect_count = 0u;
static uint32_t wifi_connect_count = 0u;
static int8_t wifi_last_rssi = 0;
static esp_timer_handle_t wifi_retry_timer = NULL;

static void wifi_retry_timer_cb(void *arg)
{
   if ((wifi_event_group != NULL) && ((xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) == 0u))
   {
       ESP_LOGW(TAG, "Retrying WiFi connection after disconnect...");
       esp_wifi_connect();
   }
}

static void wifi_start_retry_timer(void)
{
   if (wifi_retry_timer != NULL)
   {
      esp_timer_start_periodic(wifi_retry_timer, 5000000ULL);
   }
}

static void wifi_stop_retry_timer(void)
{
   if (wifi_retry_timer != NULL)
   {
       esp_timer_stop(wifi_retry_timer);
   }
}

static void wifi_log_connection_health(void)
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
            esp_wifi_connect();
            break;
         }

         case WIFI_EVENT_STA_DISCONNECTED:
         {
            wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
            wifi_disconnect_count++;
            ESP_LOGW(TAG,
                     "WiFi disconnected #%lu, reason=%d, last_rssi=%d dBm, reconnecting...",
                     (unsigned long)wifi_disconnect_count,
                     event ? event->reason : -1,
                     wifi_last_rssi);
            wifi_start_retry_timer();
            break;
         }

         default:
            break;
      }
   }
   else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
   {
      wifi_stop_retry_timer();
      xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
      wifi_connect_count++;
      wifi_log_connection_health();
      ESP_LOGI(TAG, "Connected to WiFi -- total connects: %lu", (unsigned long)wifi_connect_count);
   }
}

void wifi_connect(void)
{
   // create WIFI event group
   wifi_event_group = xEventGroupCreate();
   ESP_ERROR_CHECK((wifi_event_group == NULL) ? ESP_FAIL : ESP_OK);
   xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

   // create WIFI reconnect timer
   const esp_timer_create_args_t timer_args =
   {
      .callback = &wifi_retry_timer_cb,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "wifi_retry"
   };
   ESP_ERROR_CHECK(esp_timer_create(&timer_args, &wifi_retry_timer));

   ESP_ERROR_CHECK(esp_netif_init());
   ESP_ERROR_CHECK(esp_event_loop_create_default());
   esp_netif_create_default_wifi_sta();

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
   wifi_config_t wifi_config =
   {
      .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS }
   };
   ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
   ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
   ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
   ESP_ERROR_CHECK(esp_wifi_start());
   xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
}

void wifi_reset(void)
{
   wifi_stop_retry_timer();
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

void start_mdns_service(void)
{
    mdns_init();
    mdns_hostname_set(ESP32_HOST_NAME);         // Hostname: <host_name>.local
    mdns_instance_name_set("ESP32 Web");

    // Optional: Dienst registrieren
    mdns_service_add("WebServer", "_http", "_tcp", 80, NULL, 0);
}

void stop_mdns_service(void)
{
   mdns_free();
}