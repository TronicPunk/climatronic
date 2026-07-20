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

#include "sensor_data.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define PROJECT_NAME    "Climatronic" // char[33]

#define VERSION_MAJOR   0  // software version
#define VERSION_MINOR   01 // software minor version
#define VERSION_RELEASE 0  // software release

#define STR(s)          #s     // the C preprocessor need two steps
#define XSTR(s)         STR(s) // to stringify a macro...
#define VERSION_STRING  "V"XSTR(VERSION_MAJOR)"."XSTR(VERSION_MINOR)"r"XSTR(VERSION_RELEASE)

#define PWM_STEP              50


/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static void m_menu_main(void);
static void m_menu_sys(void);
static void m_toggle_usens_en(void);
static void m_menu_eeprom(void);
static int32_t m_test_read_eeprom(void);
static int32_t m_test_write_eeprom_addr(void);
static int32_t m_test_write_eeprom_rand(void);
static int32_t m_test_clear_eeprom(void);
static int32_t m_test_write_eeprom_pattern(const uint32_t ou32_Pattern);
static int32_t m_test_input(void);
static int32_t m_test_output(void);
static void m_test_output_print_channel(const uint32_t ou32_Channel, const uint8_t ou8_EditFlag);
static void m_test_output_disable_all(const uint32_t ou32_EditChannel);


/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static const char *TAG = "climatronic";

const T_Menu gt_MenuEeprom =
{
   MENU_MAGIC,    // menu structure identification
   m_menu_eeprom, // show menu function
   NULL,          // menu exit function
   4,             // number of menu entries
   {
      (void *)&m_test_read_eeprom,        // dump eeprom data
      (void *)&m_test_write_eeprom_addr,  // write address data
      (void *)&m_test_write_eeprom_rand,  // write random data
      (void *)&m_test_clear_eeprom,       // clear EEPROM (fill with 0xFF)
   }
};

const T_Menu gt_MenuSys =
{
   MENU_MAGIC,    // menu structure identification
   m_menu_sys,    // show menu function
   NULL,          // menu exit function
   2,             // number of menu entries
   {
      (void *)&m_toggle_usens_en,   // toggle the sensor supply
      (void *)&gt_MenuEeprom,       // eeprom test functions
   }
};

// main menu data structure
const T_Menu gt_MenuMain =
{
   MENU_MAGIC,  // menu structure identification
   m_menu_main, // show menu function
   NULL,        // menu exit function
   3,           // number of menu entries
   {
      (void *)&gt_MenuSys,    // submenu system test
      (void *)&m_test_input,  // submenu input test
      (void *)&m_test_output  // function output test
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

static void m_menu_sys(void)
{
   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" System functions\r\n");
   printf("------------------------------------------\r\n\n");
   printf(" [1]  Toggle Sensor Supply\r\n");
   printf("      Usens  = %lu mV\r\n", adc_read_mv(ADC_USENS));
   printf("      Button = 0x%02lX\r\n", io_get_switch_button());
   printf(" [2]  Test EEPROM\r\n\n");
   printf("[ESC] return to previous menu\r\n");
}

static void m_menu_eeprom(void)
{
   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" EEPROM test functions\r\n");
   printf("------------------------------------------\r\n\n");
   printf(" [1]  Read EEPROM\r\n");
   printf(" [2]  Write EEPROM with address data\r\n");
   printf(" [3]  Write EEPROM with random data\r\n");
   printf(" [4]  Clear EEPROM (fill with 0xFF)\r\n\n");
   printf("[ESC] return to previous menu\r\n");
}

//-----------------------------------------------------------------------------
/*!
   \brief    read and dump EEPROM data

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_test_read_eeprom(void)
{
   const uint32_t u32_DumpLines = 16u;
   const uint32_t u32_DumpLineSize = 16u;
   const uint32_t u32_DumpSize = u32_DumpLines * u32_DumpLineSize;
   uint8_t u8_Key = 0u;

   clr_scr();
   printf("-------------------------------------------------------\r\n");
   printf(" Read EEPROM data\r\n");
   printf("-------------------------------------------------------\r\n\n");

   serial_clear_rx_buf(); // clear RX buffer

   if (iic_eeprom_is_ready() == ESP_OK)
   {
      uint8_t au8_EepromBuffer[u32_DumpSize];
      uint32_t u32_BufferAddr = 0u;

      printf(" [ESC] Terminate, other key = next 256byte\r\n\n");

      while (u8_Key != 0x1Bu)
      {
         // read memory
         esp_err_t t_Err = iic_eeprom_read(u32_BufferAddr, u32_DumpSize, au8_EepromBuffer);
         // dump memory
         set_cursor(0u, 7u);
         for (uint32_t u32_Line = 0u; u32_Line < u32_DumpLines; u32_Line++)
         {
            const uint32_t u32_LineOffset = u32_Line * u32_DumpLineSize;

            printf("0x%04lX: ", u32_BufferAddr + u32_LineOffset);
            if (t_Err == ESP_OK)
            {
               for (uint32_t u32_Idx = 0u; u32_Idx < u32_DumpLineSize; u32_Idx++)
               {
                  printf("%02X ", au8_EepromBuffer[u32_LineOffset + u32_Idx]);
               }
               printf("\r\n");
            }
            else
            {
               printf("EEPROM read error                    \r\n");
            }
         }

         u32_BufferAddr += u32_DumpSize;
         if (u32_BufferAddr >= IIC_EEPROM_SIZE)
         {
            u32_BufferAddr = 0u;
         }

         u8_Key = get_key();
      }
   }
   else
   {
      printf(" EEPROM not ready, press any key...\r\n\n");
      get_key();
   }

   return 1;
}

//-----------------------------------------------------------------------------
/*!
   \brief    fill EEPROM with address data

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_test_write_eeprom_addr(void)
{
   return m_test_write_eeprom_pattern(0x00010000uL);
}

//-----------------------------------------------------------------------------
/*!
   \brief    fill EEPROM with random data

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_test_write_eeprom_rand(void)
{
   return m_test_write_eeprom_pattern(0x00010001uL);
}

//-----------------------------------------------------------------------------
/*!
   \brief    fill EEPROM with 0xFF data

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_test_clear_eeprom(void)
{
   return m_test_write_eeprom_pattern(0x0000FFFFuL);
}

//-----------------------------------------------------------------------------
/*!
   \brief    fill EEPROM with random data

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_test_write_eeprom_pattern(const uint32_t ou32_Pattern)
{
   const uint32_t u32_BufferSize = 256u;
   esp_err_t t_Err;

   clr_scr();
   printf("-------------------------------------------------------\r\n");
   if (ou32_Pattern > 0x00010000u)
   {
      printf(" Write EEPROM with random data\r\n");
   }
   else if (ou32_Pattern == 0x00010000u)
   {
      printf(" Write EEPROM with address data\r\n");
   }
   else
   {
      printf(" Write EEPROM with 0x%04lX data\r\n", ou32_Pattern);
   }
   printf("-------------------------------------------------------\r\n\n");

   serial_clear_rx_buf(); // clear RX buffer

   if (iic_eeprom_is_ready() == ESP_OK)
   {
      uint8_t au8_EepromBuffer[u32_BufferSize];
      uint32_t u32_BufferAddr = 0u;
      uint32_t u32_Line = 4u;

      srand((unsigned int)esp_timer_get_time());

      while (u32_BufferAddr < IIC_EEPROM_SIZE)
      {
         set_cursor(0u, u32_Line);
         printf(" write address 0x%04lX: ", u32_BufferAddr);
         // setup data buffer
         for (uint32_t u32_Idx=0; u32_Idx < u32_BufferSize; u32_Idx += 2u)
         {
            uint16_t u16_Pattern;

            if (ou32_Pattern < 0x10000u)
            {
               u16_Pattern = (uint16_t)ou32_Pattern;
            }
            else if (ou32_Pattern == 0x10000u)
            {
               u16_Pattern = u32_BufferAddr+u32_Idx;
            }
            else
            {
               u16_Pattern = (uint16_t)(rand() % 0x10000u);
            }

            au8_EepromBuffer[u32_Idx] = (uint8_t)(u16_Pattern & 0xFFu);
            au8_EepromBuffer[u32_Idx+1u] = (uint8_t)((u16_Pattern >> 8u) & 0xFFu);
         }

         t_Err = iic_eeprom_write(u32_BufferAddr, u32_BufferSize, au8_EepromBuffer);

         if (t_Err == ESP_OK)
         {
            printf("OK");
         }
         else
         {
            printf("error %d", t_Err);
            u32_Line++;
         }

         u32_BufferAddr += u32_BufferSize;
      }

      printf("\r\n\n press any key...");
      get_key();
   }
   else
   {
      printf(" EEPROM not ready, press any key...\r\n\n");
      get_key();
   }

   return 1;
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


//-----------------------------------------------------------------------------
/*!
   \brief    set outputs manually

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_test_output(void)
{
   uint32_t u32_EditChannel = 0;
   int32_t s32_Pwm;
   uint8_t u8_Key = 0u;

   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" manual output test\r\n");
   printf("------------------------------------------\r\n");
   m_test_output_disable_all(u32_EditChannel);
   printf("\n------------------------------------------------------------\r\n");
   printf(" [1] select out channel-\r\n");
   printf(" [2] select out channel+\r\n");
   printf(" [3] set PWM-\r\n");
   printf(" [4] set PWM+\r\n");
   printf(" [5] disable all outputs\r\n\n");
   printf("[ESC] terminate test");

   serial_clear_rx_buf(); // clear RX buffer

   while (u8_Key != 0x1Bu)
   {
      if (u8_Key == '1')
      {
         m_test_output_print_channel(u32_EditChannel, false);
         u32_EditChannel = (u32_EditChannel > 0u) ? (u32_EditChannel - 1u) : (OUTPUT_NUM_CHANNEL_MAX -1u);
      }
      if (u8_Key == '2')
      {
         m_test_output_print_channel(u32_EditChannel, false);
         u32_EditChannel++;
         if (u32_EditChannel >= OUTPUT_NUM_CHANNEL_MAX)
         {
            u32_EditChannel = 0u;
         }
      }
      if (u8_Key == '3')
      {
         output_get_pwm(u32_EditChannel, &s32_Pwm);
         s32_Pwm = (s32_Pwm >= PWM_STEP) ? (s32_Pwm - PWM_STEP) : OUTPUT_PWM_MAX;
         output_set_pwm(u32_EditChannel, s32_Pwm);
      }
      if (u8_Key == '4')
      {
         output_get_pwm(u32_EditChannel, &s32_Pwm);
         s32_Pwm += PWM_STEP;
         if (s32_Pwm > OUTPUT_PWM_MAX)
         {
            s32_Pwm = 0;
         }
         output_set_pwm(u32_EditChannel, s32_Pwm);
      }
      if (u8_Key == '5')
      {
         m_test_output_disable_all(u32_EditChannel);
      }

      m_test_output_print_channel(u32_EditChannel, true);

      u8_Key = get_key();
   }

   // disable all outputs
   m_test_output_disable_all(u32_EditChannel);

   return 1;
}

static void m_test_output_print_channel(const uint32_t ou32_Channel, const uint8_t ou8_EditFlag)
{
   int32_t s32_Pwm;
   static const char* as_EditFlag[2] = { "   " , " * " };
   uint8_t u8_FlagIdx = (ou8_EditFlag >= 1u) ? 1u : 0u;

   output_get_pwm(ou32_Channel, &s32_Pwm);
   set_cursor(0u, ou32_Channel + 5u);
   printf("%sOUTPUT%lu: PWM = %4ld %%\r\n", as_EditFlag[u8_FlagIdx], ou32_Channel, (s32_Pwm + 5)/10);
}

static void m_test_output_disable_all(const uint32_t ou32_EditChannel)
{
   for (uint32_t u32_Channel = 0u; u32_Channel < OUTPUT_NUM_CHANNEL_MAX; u32_Channel++)
   {
      uint8_t u8_EditFlag = (u32_Channel == ou32_EditChannel) ? 1u : 0u;
      output_set_pwm(u32_Channel, 0);
      m_test_output_print_channel(u32_Channel, u8_EditFlag);
   }
}

static void m_toggle_usens_en(void)
{
   uint32_t u32_UsensEn = io_get_sensor_supply_state();
    /* Toggle the USENS_EN signal */
   u32_UsensEn = (u32_UsensEn != 0) ? 0u : 1u;
   io_set_sensor_supply_state(u32_UsensEn);
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
