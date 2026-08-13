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
static int32_t m_test_input(void);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static const char *TAG = "climatronic";

// main menu data structure
const T_Menu gt_MenuMain =
{
   MENU_MAGIC,  // menu structure identification
   m_menu_main, // show menu function
   NULL,        // menu exit function
   3,           // number of menu entries
   {
      (void *)&gt_MenuSystemSetup,  // submenu system test
      (void *)&m_test_input,        // submenu input test
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
   printf(" [2]  Inputs (%u sensors found)\r\n", u16_SensorCnt);
   printf(" [3]  Outputs\r\n");
}

//-----------------------------------------------------------------------------
/*!
   \brief    read all sensor information

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_test_input(void)
{
   uint16_t u16_SensorCnt = sensor_get_count();
   uint16_t u16_SensorIdx;
   T_sensor_info t_SensorInfo;
   uint8_t u8_Key = 0u;

   clr_scr();
   printf("-------------------------------------------------------\r\n");
   printf(" Show Sensor data ([ESC] terminate, [c] calibrate CO2)\r\n");
   printf("-------------------------------------------------------\r\n\n");
   printf("Sensor Type   Name                     | T [°C] | P [hPa] | Hum [%%] | CO2 [ppm] | sensor state | data age [s]\r\n");
   printf("---------------------------------------------------------------------------------------------------------------\r\n");

   for (u16_SensorIdx = 0u; u16_SensorIdx < u16_SensorCnt; u16_SensorIdx++)
   {
      (void)sensor_get_info(u16_SensorIdx, &t_SensorInfo);

      printf("%2u %s %s", u16_SensorIdx, t_SensorInfo.pt_Sensor->s_Type, t_SensorInfo.s_Name);
      set_cursor(40u, u16_SensorIdx + 7u);
      printf("|  ---.- |  ----.- |  ---.-  |   -----   |  0x00000000  |\r\n");
   }

   serial_clear_rx_buf(); // clear RX buffer

   while (u8_Key != 0x1Bu)
   {
      delay_us(1000000u);

      if ((u8_Key == 'c') || (u8_Key == 'C'))
      {
         // calibrate all CO2 sensors
         for (u16_SensorIdx = 0; u16_SensorIdx < u16_SensorCnt; u16_SensorIdx++)
         {
            (void)sensor_get_info(u16_SensorIdx, &t_SensorInfo);

            if ((t_SensorInfo.pt_Sensor->u16_SensorCaps & SENSOR_CAP_CO2) != 0u)
            {
               uint32_t u32_Parameter = 400u; // calibrate CO2 sensors to 400ppm
               (void)sensor_command(u16_SensorIdx, SENSOR_COMMAND_CALIBRATE_CO2, &u32_Parameter);
            }
         }
      }

      // read data from all connected sensors
      for (u16_SensorIdx = 0; u16_SensorIdx < u16_SensorCnt; u16_SensorIdx++)
      {
         T_sensor_data t_SensorData;
         uint32_t u32_SensorState;

         (void)sensor_get_data(u16_SensorIdx, &t_SensorData);           // read sensor data
         (void)sensor_command(u16_SensorIdx, SENSOR_COMMAND_GET_STATE, &u32_SensorState); // read sensor state

         // print sensor data
         set_cursor(43u, u16_SensorIdx + 7u);
         if (t_SensorData.f32_Temperature != SENSOR_VALUE_INVALID)
         {
            printf("%5.1f", t_SensorData.f32_Temperature);
         }
         else
         {
            printf("---.-");
         }

         set_cursor(52u, u16_SensorIdx + 7u);
         if (t_SensorData.f32_Pressure != SENSOR_VALUE_INVALID)
         {
            printf("%6.1f", t_SensorData.f32_Pressure);
         }
         else
         {
            printf("----.-");
         }

         set_cursor(62u, u16_SensorIdx + 7u);
         if (t_SensorData.f32_Humidity != SENSOR_VALUE_INVALID)
         {
            printf("%5.1f", t_SensorData.f32_Humidity);
         }
         else
         {
            printf("---.-");
         }

         set_cursor(73u, u16_SensorIdx + 7u);
         if (t_SensorData.f32_CO2 != SENSOR_VALUE_INVALID)
         {
            printf("%5.0f", t_SensorData.f32_CO2);
         }
         else
         {
            printf("-----");
         }

         set_cursor(86u, u16_SensorIdx + 7u);
         printf("%08lX", u32_SensorState);

         set_cursor(99u, u16_SensorIdx + 7u);
         if (t_SensorData.u32_TimeStampSec != SENSOR_TIMESTAMP_INVALID)
         {
            const uint32_t u32_AgeSec = get_time_sec() - t_SensorData.u32_TimeStampSec;
            printf("%8lu", u32_AgeSec);
         }
         else
         {
            printf("     ---");
         }
      }

      u8_Key = get_key_nowait();
   }

   return 1;
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
