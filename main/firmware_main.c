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
#include "sensor.h"
#include "output.h"
#include "timer.h"
#include "wifi_connect.h"
#include "webserver.h"
#include "menu.h"

#include "system_setup.h"
#include "sensor_data.h"
#include "sensor_setup.h"
#include "output_setup.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define PROJECT_NAME    "Climatronic" // char[33]

#define VERSION_MAJOR   0  // software version
#define VERSION_MINOR   01 // software minor version
#define VERSION_RELEASE 0  // software release

#define STR(s)          #s     // the C preprocessor need two steps
#define XSTR(s)         STR(s) // to stringify a macro...
#define VERSION_STRING  "V"XSTR(VERSION_MAJOR)"."XSTR(VERSION_MINOR)"r"XSTR(VERSION_RELEASE)


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
   printf(" Climatronic Testsoftware V%d.%02dr%d / %s\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_RELEASE, __DATE__);
   printf("------------------------------------------\r\n\n");
   printf(" [1]  System\r\n");
   printf(" [2]  Sensor values (%u sensors found)\r\n", u16_SensorCnt);
   printf(" [3]  Sensor setup\r\n");
   printf(" [4]  Outputs\r\n");
}


void app_main(void)
{
   esp_err_t err;

   // force linkage of sensor components
   __force_link_bme280();
   __force_link_bme68x();
   __force_link_scd4x();

   io_init();        // init IO signals
   ESP_ERROR_CHECK(nvs_flash_init());
   serial_init();

   // init the web interface
   wifi_connect();
   start_mdns_service();
   start_webserver();

   iic_init();          // init the IIC interface
   iic_mux_init();      // init the IIC multiplexer
   iic_eeprom_init();   // init the IIC eeprom

   err = output_init();
   if (err != ESP_OK)
   {
      ESP_LOGI(TAG, "output_init error %d\r\n", err);
      get_key();
   }

   err = sensor_init_task();
   if (err != ESP_OK)
   {
      ESP_LOGI(TAG, "no sensor connected!\r\n", err);
      get_key();
   }

   while (true)
   {
      menu_entry(&gt_MenuMain);
      delay_us(50000u);
   }
}
