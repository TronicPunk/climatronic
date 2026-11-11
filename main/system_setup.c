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
#include "system_data.h"
#include "device_data.h"
#include "iic_eeprom.h"
#include "inout.h"
#include "timer.h"
#include "system.h"
#include "esp_wifi.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static void m_menu_system(void);
static void m_toggle_usens_en(void);
static int32_t m_set_led(void);
static int32_t m_setup_i2c_bus_freq(void);
static int32_t m_wifi_setup(void);
static esp_err_t m_edit_string(char * const oac_String, const uint32_t ou32_Len, const uint32_t ou32_X, const uint32_t ou32_Y);
static void m_bssid_to_string(const uint8_t * const opu8_BSSID, char * const oac_BSSID_String);
static esp_err_t m_wifi_scan_aps(char * const oac_SSID, uint8_t * const oau8_BSSID);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

// system setup menu data structure
const T_Menu gt_MenuSystemSetup =
{
   MENU_MAGIC,       // menu structure identification
   &m_menu_system,   // show system setup menu
   NULL,             // menu exit function
   6,                // number of menu entries
   {
      (void *)&m_toggle_usens_en,         // toggle signal usens_en
      (void *)&m_set_led,                 // set LEDs
      (void *)&gt_MenuEepromTest,         // menu EEPROM test
      (void *)&m_setup_i2c_bus_freq,      // setup I2C bus frequency
      (void *)&m_wifi_setup,              // WIFI setup
      (void *)&system_sw_reset,           // ECU software reset
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
   printf(" [2]  Set LED\r\n");
   printf(" [3]  Test EEPROM\r\n");
   printf(" [4]  Set I2C bus frequency\r\n");
   printf(" [5]  WIFI Setup\r\n");
   printf(" [6]  Reset\r\n\n");
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


//-----------------------------------------------------------------------------
/*!
   \brief    set LEDss manually

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_set_led(void)
{
   char * as_LEDs[STATUS_LED_COUNT] = { "SENS", "WLAN", " CPU" };
   uint32_t u32_LED = STATUS_LED_SENSOR;
   uint32_t u32_Color = io_get_led_color(u32_LED);
   uint8_t u8_Red = (uint8_t)((u32_Color & STATUS_LED_RED_MSK) >> STATUS_LED_RED_BITPOS);
   uint8_t u8_Green = (uint8_t)((u32_Color & STATUS_LED_GREEN_MSK) >> STATUS_LED_GREEN_BITPOS);
   uint8_t u8_Blue = (uint8_t)((u32_Color & STATUS_LED_BLUE_MSK) >> STATUS_LED_BLUE_BITPOS);
   uint8_t u8_Key = 0u;

   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" set LEDs\r\n");
   printf("------------------------------------------\r\n\n");
   printf(" [1]  select LED  [SENS]\r\n");
   printf("[r,R] set red      [  0]\r\n");
   printf("[g,G] set green    [  0]\r\n");
   printf("[b,B] set blue     [  0]\r\n");
   printf("      color      #000000\r\n\n");
   printf("[ESC] terminate setup");

   serial_clear_rx_buf(); // clear RX buffer

   while (u8_Key != 0x1Bu)
   {
      if (u8_Key == '1')
      {
         u32_LED++;
         if (u32_LED >= STATUS_LED_COUNT)
         {
            u32_LED = STATUS_LED_SENSOR;
         }
         u32_Color = io_get_led_color(u32_LED);
         u8_Red = (uint8_t)((u32_Color & STATUS_LED_RED_MSK) >> STATUS_LED_RED_BITPOS);
         u8_Green = (uint8_t)((u32_Color & STATUS_LED_GREEN_MSK) >> STATUS_LED_GREEN_BITPOS);
         u8_Blue = (uint8_t)((u32_Color & STATUS_LED_BLUE_MSK) >> STATUS_LED_BLUE_BITPOS);
      }
      if ((u8_Key == 'r') || (u8_Key == 'R'))
      {
         if (u8_Key == 'r')
         {
            u8_Red--;
         }
         else
         {
            u8_Red++;
         }
         u32_Color &= (uint32_t)(~STATUS_LED_RED_MSK);
         u32_Color |= ((uint32_t)u8_Red << STATUS_LED_RED_BITPOS);
      }
      if ((u8_Key == 'g') || (u8_Key == 'G'))
      {
         if (u8_Key == 'g')
         {
            u8_Green--;
         }
         else
         {
            u8_Green++;
         }
         u32_Color &= (uint32_t)(~STATUS_LED_GREEN_MSK);
         u32_Color |= ((uint32_t)u8_Green << STATUS_LED_GREEN_BITPOS);
      }
      if ((u8_Key == 'b') || (u8_Key == 'B'))
      {
         if (u8_Key == 'b')
         {
            u8_Blue--;
         }
         else
         {
            u8_Blue++;
         }
         u32_Color &= (uint32_t)(~STATUS_LED_BLUE_MSK);
         u32_Color |= ((uint32_t)u8_Blue << STATUS_LED_BLUE_BITPOS);
      }

      set_cursor(20u, 5u);
      printf("%s", as_LEDs[u32_LED]);
      set_cursor(21u, 6u);
      printf("%3u", u8_Red);
      set_cursor(21u, 7u);
      printf("%3u", u8_Green);
      set_cursor(21u, 8u);
      printf("%3u", u8_Blue);
      set_cursor(19u, 9u);
      printf("%06lX", u32_Color);

      io_set_led(u32_LED, u32_Color);

      u8_Key = get_key();
   }

   return 1;
}

static int32_t m_wifi_setup(void)
{
   T_device_data t_DeviceData;
   char ac_String[64u];
   char ac_BSSID[18u];
   esp_err_t t_Err;
   uint8_t u8_Key = 0u;
   bool q_SetupModified = true;

   serial_clear_rx_buf(); // clear RX buffer
   device_data_get(&t_DeviceData);

   while (u8_Key != 0x1Bu)
   {
      m_bssid_to_string(t_DeviceData.au8_BSSID, ac_BSSID);

      clr_scr();
      printf("------------------------------------------\r\n");
      printf(" WIFI setup (NVM data)\r\n");
      printf("------------------------------------------\r\n\n");
      printf(" [1] edit hostname    %s\r\n", t_DeviceData.ac_HostName);
      printf("     WIFI SSID        %s\r\n", t_DeviceData.ac_WiFiSSID);
      printf("     WIFI BSSID       %s\r\n", ac_BSSID);
      printf(" [2] set  WIFI PW     ********\r\n");
      printf(" [3] scan WIFI APs for SSID\r\n\n");
      printf("[ESC] terminate setup\r\n\n");

      while (!((u8_Key == 0x1Bu) || ((u8_Key >= '1') && (u8_Key <= '3'))))
      {
         wifi_ap_record_t ap_info;
         memset(&ap_info, 0, sizeof(ap_info));
         set_cursor(1u, 13u);
         if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
         {
            printf("WiFi health: SSID=%s, RSSI=%d dBm, ch=%d, auth=%d   \r\n\n", ap_info.ssid, ap_info.rssi,
                  ap_info.primary, ap_info.authmode);
         }
         else
         {
            printf("WiFi health: No AP info available yet\r\n\n");
         }

         delay_us(1000000u);// refresh after 1sec
         u8_Key = get_key_nowait();
      }

      if (u8_Key == '1')
      {
         t_Err = m_edit_string(ac_String, DEVICE_HOST_NAME_MAX, 23u, 5u);
         if (t_Err == ESP_OK)
         {
            device_data_set_hostname(ac_String);
            q_SetupModified = true;
         }
      }
      if (u8_Key == '2')
      {
         t_Err = m_edit_string(ac_String, WIFI_PASSWORD_MAX, 23u, 8u);
         if (t_Err == ESP_OK)
         {
            device_data_set_wifipw(ac_String);
            q_SetupModified = true;
         }
      }
      if (u8_Key == '3')
      {
         uint8_t au8_BSSID[WIFI_BSSID_SIZE];
         t_Err = m_wifi_scan_aps(ac_String, au8_BSSID);
         if (t_Err == ESP_OK)
         {
            device_data_set_bssid(au8_BSSID);
            device_data_set_ssid(ac_String);
            q_SetupModified = true;
         }
      }

      if (q_SetupModified == true)
      {
         device_data_store();
         q_SetupModified = false;
      }

      u8_Key = (u8_Key != 0x1Bu) ? 0u : u8_Key;
   }

   return 1;
}

static esp_err_t m_edit_string(char * const oac_String, const uint32_t ou32_Len, const uint32_t ou32_X, const uint32_t ou32_Y)
{
   memset(oac_String, ' ', ou32_Len);
   oac_String[ou32_Len] = '\0';
   set_cursor(ou32_X, ou32_Y);
   printf("%s",oac_String);
   set_cursor(ou32_X, ou32_Y);
   return get_string((uint8_t *)oac_String, ou32_Len);
}

static void m_bssid_to_string(const uint8_t * const opu8_BSSID, char * const oac_BSSID_String)
{
   sprintf(oac_BSSID_String, "%02X:%02X:%02X:%02X:%02X:%02X", opu8_BSSID[0], opu8_BSSID[1], opu8_BSSID[2],
           opu8_BSSID[3], opu8_BSSID[4], opu8_BSSID[5]);
}

static esp_err_t m_wifi_scan_aps(char * const oac_SSID, uint8_t * const oau8_BSSID)
{
   esp_err_t t_Err = ESP_FAIL;
   wifi_ap_record_t ap_records[5];
   uint16_t ap_count;
   bool q_ClearApList = false;

   printf("Start scanning for APs...\r\n");
   ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
   ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
   if (ap_count > 5u)
   {
      ap_count = 5u;
      q_ClearApList = true;
   }
   if (ap_count != 0u)
   {
      uint8_t u8_Key = 0u;

      ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_records));
      if (q_ClearApList)
      {
         ESP_ERROR_CHECK(esp_wifi_clear_ap_list());
      }

      printf("%u AP found:\r\n", ap_count);
      for (int i = 0; i < ap_count; i++)
      {
         char ac_BSSID[18u];
         m_bssid_to_string(ap_records[i].bssid, ac_BSSID);
         printf("[%u] BSSID: %s, SSID: %s, RSSI: %d\r\n", i+1, ac_BSSID, ap_records[i].ssid, ap_records[i].rssi);
      }
      printf("Select AP by [index] or [ESC] to terminate\r\n");

      while (u8_Key != 0x1Bu)
      {
         if ((u8_Key >= '1') && (u8_Key <= '9'))
         {
            const int i = u8_Key - '1';
            if (i < ap_count)
            {
               strncpy(oac_SSID, (const char *)ap_records[i].ssid, WIFI_SSID_MAX);
               oac_SSID[WIFI_SSID_MAX] = '\0';
               memcpy(oau8_BSSID, ap_records[i].bssid, WIFI_BSSID_SIZE);
               t_Err = ESP_OK;
               break;
            }
         }
         u8_Key = get_key();
      }
   }
   else
   {
      printf("No AP found, press any key...\r\n");
      get_key();
   }

   return t_Err;
}


static int32_t m_setup_i2c_bus_freq(void)
{
   const char * as_I2cBusFreq[2] = { "100000", "400000" };
   uint32_t u32_I2cBus;
   uint8_t u8_I2cBusFreq;
   uint8_t u8_Key = 0u;

   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" Setup I2C bus frequency\r\n");
   printf("------------------------------------------\r\n\n");
   for (u32_I2cBus = 0; u32_I2cBus < I2C_BUS_COUNT; u32_I2cBus++)
   {
      const uint32_t u32_I2cBusNum = u32_I2cBus+1u;
      system_data_get_i2c_freq(u32_I2cBus, &u8_I2cBusFreq);
      printf(" [%lu]  Change I2C bus_%lu frequency [%s]\r\n", u32_I2cBusNum, u32_I2cBusNum, as_I2cBusFreq[u8_I2cBusFreq]);
   }
   printf("\n[ESC] Terminate\n");

   while (u8_Key != 0x1Bu)
   {
      if ((u8_Key >= '1') && (u8_Key < '1' + I2C_BUS_COUNT))
      {
         u32_I2cBus = u8_Key - '1';
         system_data_get_i2c_freq(u32_I2cBus, &u8_I2cBusFreq);
         u8_I2cBusFreq = (u8_I2cBusFreq == I2C_BUS_FREQ_100K) ? I2C_BUS_FREQ_400K : I2C_BUS_FREQ_100K;
         system_data_set_i2c_freq(u32_I2cBus, u8_I2cBusFreq);
         system_data_store();
         set_cursor(34u, 5u+u32_I2cBus);
         printf("%s\n", as_I2cBusFreq[u8_I2cBusFreq]);
      }

      delay_us(500000u);// refresh after 0.5sec
      u8_Key = get_key_nowait();
   }

   return 1;
}
