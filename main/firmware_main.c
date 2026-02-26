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
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "iic.h"
#include "iic_mux.h"
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

#define BLINK_GPIO            GPIO_NUM_2
#define CONFIG_BLINK_PERIOD   1000
#define PWM_STEP              50


/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static void m_menu_main(void);
static void m_menu_sub(void);
static int32_t m_test_input(void);
static int32_t m_test_output(void);
static void m_test_output_print_channel(const uint32_t ou32_Channel, const uint8_t ou8_EditFlag);
static void m_test_output_disable_all(const uint32_t ou32_EditChannel);
static void m_configure_led(void);
static void m_toggle_led(void);


/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static const char *TAG = "climatronic";
static uint8_t s_led_state = 0;

const T_Menu gt_MenuSub =
{
   MENU_MAGIC,    // menu structure identification
   m_menu_sub,    // show menu function
   NULL,          // menu exit function
   1,             // number of menu entries
   {
      (void *)&m_toggle_led,   // submenu system test
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
      (void *)&gt_MenuSub,    // submenu system test
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

static void m_menu_sub(void)
{
   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" System functions\r\n");
   printf("------------------------------------------\r\n\n");
   printf(" [1]  Toggle LED\r\n\n");
   printf("[ESC] return to previous menu\r\n");
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


static void m_configure_led(void)
{
    s_led_state = 0;
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void m_toggle_led(void)
{
    /* Toggle the LED state */
    s_led_state = !s_led_state;
   /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

void app_main(void)
{
   esp_err_t err;

   // force linkage of sensor components
   __force_link_bme280();
   __force_link_bme68x();
   __force_link_scd4x();

   ESP_ERROR_CHECK(nvs_flash_init());
   m_configure_led();
   serial_init();

   // init the web interface
   wifi_connect();
   start_mdns_service();
   start_webserver();

   iic_init();       // init the IIC interface
   iic_mux_init();   // init the IIC multiplexer

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
