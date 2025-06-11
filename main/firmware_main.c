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
static int32_t m_func_invalid(void);
static int32_t m_test_output(void);
static void m_test_output_print_channel(const uint32_t ou32_Channel, const uint8_t ou8_EditFlag);
static void m_test_output_disable_all(const uint32_t ou32_EditChannel);
static void m_configure_led(void);
static void m_blink_led(void);


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
      (void *)&m_blink_led,   // submenu system test
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
      (void *)&gt_MenuSub,       // submenu system test
      (void *)&m_func_invalid,   // submenu input test
      (void *)&m_test_output     // function output test
   }
};


/* -- Implementation ------------------------------------------------------------------------------------------------ */

static void m_menu_main(void)
{
   // display main menu
   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" Climatronic Testsoftware V%d.%02dr%d / %s\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_RELEASE, __DATE__);
   printf("------------------------------------------\r\n\n");
   printf(" [1]  System\r\n");
   printf(" [2]  Inputs\r\n");
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


static int32_t m_func_invalid(void)
{
   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" Function not implemented\r\n");
   printf("------------------------------------------\r\n");

   return 0;
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
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    s_led_state = 0;
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void m_blink_led(void)
{
    /* Toggle the LED state */
    s_led_state = !s_led_state;
   /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

void app_main(void)
{
   esp_err_t err = ESP_FAIL;
   T_sensor_data t_SensorData;
   uint8_t u8_SensorCnt = sensor_get_count();

   // force linkage of sensor components
   __force_link_bme280();
   __force_link_bme68x();
   __force_link_scd4x();

   ESP_ERROR_CHECK(nvs_flash_init());
   wifi_connect();
   start_mdns_service();
   start_webserver();

   m_configure_led();
   serial_init();

   iic_init();       // init the IIC interface
   iic_mux_init();   // init the IIC multiplexer

   err = output_init();
   if (err != ESP_OK)
   {
      ESP_LOGI(TAG, "output_init error %d", err);
      get_key();
   }


   printf("\r\n");
   printf("Sensor Interface: %u sensor(s) found\r\n", u8_SensorCnt);
   // init all sensors
   for (uint8_t u8_SensorIdx =0; u8_SensorIdx < u8_SensorCnt; u8_SensorIdx++)
   {
      const T_sensor_info * pt_SensorInfo;
      sensor_get_info(u8_SensorIdx, &pt_SensorInfo);
      printf("Sensor %u: Type %s at I2C bus %u initialization ", u8_SensorIdx, pt_SensorInfo->pt_Sensor->s_Type, pt_SensorInfo->u8_IicBus);
      err = sensor_init(u8_SensorIdx);
      if (err == ESP_OK)
      {
         printf("OK\r\n");
      }
      else
      {
         printf("error!\r\n");
      }
   }

   if (err == ESP_OK)
   {
      printf("\r\n");
      while (1)
      {
         bool q_SCD4x_Ready = false;
         delay_us(1000000u);

         for (uint32_t i=0; i<2; i++)
         {
            const T_sensor_info * pt_SensorInfo;
            sensor_get_info(i, &pt_SensorInfo);
            err = sensor_process_data(i);
            if ((err != ESP_ERR_NOT_FINISHED) && (err != ESP_OK))
            {
               printf("%s (%s): ", pt_SensorInfo->s_Name, pt_SensorInfo->pt_Sensor->s_Type);
               printf("sensor_process_data(%lu) error = %u\r\n\n", i, err);
            }

            if ((i == 1) && (err == ESP_OK))
            {
               q_SCD4x_Ready = true;
            }
         }

         if (q_SCD4x_Ready == true)
         {
            for (uint32_t i=0; i<2; i++)
            {
               const T_sensor_info * pt_SensorInfo;
               sensor_get_info(i, &pt_SensorInfo);
               err = sensor_get_data(i, &t_SensorData);

               printf("%s (%s): ", pt_SensorInfo->s_Name, pt_SensorInfo->pt_Sensor->s_Type);
               printf("sensor_get_data(%lu) = %u\r\n", i, err);
               printf("Temperature    %5.1f °C\r\n", t_SensorData.f32_Temperature);
               printf("Pressure       %6.1f hPa\r\n", t_SensorData.f32_Pressure*0.01f);
               printf("Humidity       %5.1f %%\r\n", t_SensorData.f32_Humidity);
               printf("CO2            %5.0f ppm\r\n", t_SensorData.f32_CO2);
               printf("data age    %8lu sec\r\n\n", t_SensorData.u32_AgeSec);
            }
         }
      }
   }

/*
   err = sensor_get_data(1, &t_SensorData);
   printf("sensor_get_data(0) = %u\r\n", err);
   printf("Temperature %5.1f°C\r\n", t_SensorData.f32_Temperature);
   printf("Pressure    %6.1fhPa\r\n", t_SensorData.f32_Pressure*0.01f);
   printf("Humidity    %5.1f%%\r\n", t_SensorData.f32_Humidity);
   printf("\r\n");
*/
   while (true)
   {
      menu_entry(&gt_MenuMain);
      delay_us(50000u);      
   }
}
