/*
   Climatronic Firmware

   - Initialize the sensor measurement tasks
   - terminal IO: show sensor data
   - terminal IO: control outputs

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "inout.h"
#include "iic.h"
#include "iic_mux.h"
#include "iic_eeprom.h"
#include "wifi_data.h"
#include "mqtt_data.h"
#include "system_data.h"
#include "sensor.h"
#include "output.h"
#include "timer.h"
#include "wifi_connect.h"
#include "webserver.h"
#include "mqtt_interface.h"
#include "menu.h"
#include "firmware_version.h"

#include "system_setup.h"
#include "sensor_data.h"
#include "sensor_setup.h"
#include "output_setup.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define PROJECT_NAME    "Climatronic" // char[33]

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static void m_menu_main(void);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static const char *TAG = "climatronic";

// main menu data structure
const T_Menu gt_MenuMain =
{
   MENU_MAGIC,  // menu structure identification
   m_menu_main, // show menu function
   NULL,        // menu exit function
   4,           // number of menu entries
   {
      (void *)&gt_MenuSystemSetup,  // submenu system test
      (void *)&sensor_test,         // sensor test: show all sensor values
      (void *)&sensor_setup,        // sensor setup: edit sensor names
      (void *)&gt_MenuOutputSetup,  // submenu output
   }
};


/* -- Implementation ------------------------------------------------------------------------------------------------ */

static void m_menu_main(void)
{
   uint16_t u16_SensorCnt = sensor_get_count();

   // display main menu
   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" Climatronic Firmware %s / %s\r\n", FIRMWARE_VERSION_STRING, __DATE__);
   printf("------------------------------------------\r\n\n");
   printf(" [1]  System\r\n");
   printf(" [2]  Sensor values (%u sensors found)\r\n", u16_SensorCnt);
   printf(" [3]  Sensor setup\r\n");
   printf(" [4]  Outputs\r\n");
}


void app_main(void)
{
   esp_err_t err;
   T_wifi_data t_WiFiData;
   T_mqtt_data t_MqttData;

   // force linkage of sensor components
   __force_link_bme280();
   __force_link_bme68x();
   __force_link_scd4x();

   io_init();           // init IO
   io_set_led_off();    // disable status LEDs

   iic_init();          // init the IIC interface
   iic_mux_init();      // init the IIC multiplexer
   iic_eeprom_init();   // init the IIC eeprom

   ESP_ERROR_CHECK(nvs_flash_init());
   serial_init();

   err = wifi_data_load();
   if (err == ESP_FAIL)
   {
      ESP_LOGI(TAG, "WiFi data error %d\r\n", err);
   }
   wifi_data_get(&t_WiFiData);

   err = mqtt_data_load();
   if (err == ESP_FAIL)
   {
      ESP_LOGI(TAG, "MQTT data error %d\r\n", err);
   }
   mqtt_data_get(&t_MqttData);

   // init the web interface
   wifi_connect(&t_WiFiData);
   start_mdns_service(t_WiFiData.ac_HostName);
   start_webserver();

   err = system_data_load();
   if (err == ESP_FAIL)
   {
      ESP_LOGI(TAG, "system_data error %d\r\n", err);
   }

   err = output_init();
   if (err != ESP_OK)
   {
      ESP_LOGI(TAG, "output_init error %d\r\n", err);
   }
   err = sensor_init_task();
   if (err != ESP_OK)
   {
      ESP_LOGI(TAG, "no sensor connected!\r\n", err);
   }
   err = mqtt_client_start(&t_MqttData);
   if (err != ESP_OK)
   {
      ESP_LOGE(TAG, "MQTT initialization failed: %s", esp_err_to_name(err));
   }

   while (true)
   {
      menu_entry(&gt_MenuMain);
      delay_us(50000u);
   }
}
