//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  system_setup.c
   \brief system setup and test functions
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <stdio.h>
#include <string.h>
#include "system_setup.h"
#include "eeprom_test.h"
#include "iic_eeprom.h"
#include "inout.h"
#include "timer.h"
#include "nvs_flash.h"
#include "wifi_connect.h"
#include "webserver.h"
#include "esp_system.h"
#include "esp_wifi.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static void m_menu_system(void);
static void m_toggle_usens_en(void);
static int32_t m_wifi_connection_health(void);
static void m_sys_sw_reset(void);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

// system setup menu data structure
const T_Menu gt_MenuSystemSetup =
{
   MENU_MAGIC,       // menu structure identification
   &m_menu_system,   // show system setup menu
   NULL,             // menu exit function
   4,                // number of menu entries
   {
      (void *)&m_toggle_usens_en,         // toggle signal usens_en
      (void *)&gt_MenuEepromTest,         // menu EEPROM test
      (void *)&m_wifi_connection_health,  // WIFI connection information
      (void *)&m_sys_sw_reset,            // ECU software reset
   }
};


/* -- Implementation ------------------------------------------------------------------------------------------------ */

static void m_menu_system(void)
{
   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" System functions\r\n");
   printf("------------------------------------------\r\n\n");
   printf(" [1]  Toggle Sensor Supply\r\n");
   printf("      Usens  = %lu mV\r\n", adc_read_mv(ADC_USENS));
   printf("      Button = 0x%02lX\r\n", io_get_switch_button());
   printf(" [2]  Test EEPROM\r\n");
   printf(" [3]  WIFI Status\r\n");
   printf(" [4]  Reset\r\n\n");
   printf("[ESC] return to previous menu\r\n");
}

static void m_toggle_usens_en(void)
{
   uint32_t u32_UsensEn = io_get_sensor_supply_state();
    /* Toggle the USENS_EN signal */
   u32_UsensEn = (u32_UsensEn != 0) ? 0u : 1u;
   io_set_sensor_supply_state(u32_UsensEn);
   delay_us(50000u);
   if (u32_UsensEn != 0)
   {
      iic_eeprom_is_ready();
   }
}

static int32_t m_wifi_connection_health(void)
{
   uint8_t u8_Key = 0u;
   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" WiFi Status\r\n");
   printf("------------------------------------------\r\n\n");
   printf(" [1]  WIFI connect\r\n");
   printf("[ESC] Terminate");

   while (u8_Key != 0x1Bu)
   {
      wifi_ap_record_t ap_info;
      memset(&ap_info, 0, sizeof(ap_info));
      set_cursor(1u, 9u);
      if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
      {
         printf("WiFi health: SSID=%s, RSSI=%d dBm, ch=%d, auth=%d   \r\n\n", ap_info.ssid, ap_info.rssi,
               ap_info.primary, ap_info.authmode);
      }
      else
      {
         printf("WiFi health: No AP info available yet\r\n\n");
      }

      if (u8_Key == '1')
      {
         esp_wifi_connect(); // try to connect
      }

      delay_us(1000000u);// refresh after 1sec
      u8_Key = get_key_nowait();
   }

   return 1;
}



//-----------------------------------------------------------------------------
/*!
   \brief    do controller software reset
*/
//-----------------------------------------------------------------------------
static void m_sys_sw_reset(void)
{
   nvs_flash_deinit();
   stop_webserver();
   stop_mdns_service();
   wifi_reset();
   esp_restart();
}
