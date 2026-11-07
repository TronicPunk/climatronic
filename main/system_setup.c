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
static int32_t m_wifi_connection_health(void);

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
      (void *)&m_wifi_connection_health,  // WIFI connection information
      (void *)&system_sw_reset,            // ECU software reset
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
   printf(" [5]  WIFI Status\r\n");
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
