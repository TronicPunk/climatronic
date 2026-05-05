//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  inout.c
   \brief provide low level input/output functions
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "driver/gpio.h"
#include "led_strip.h"
#include "inout.h"
#include "output.h"
#include "adc.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define GPIO_USENS_EN      GPIO_NUM_2
#define GPIO_CAN_TX        GPIO_NUM_4
#define GPIO_CAN_RX        GPIO_NUM_15
#define GPIO_RGB_LED       GPIO_NUM_12

#define GPIO_OUT_MASK      ((1ULL << GPIO_OUT1) | (1ULL << GPIO_OUT2) | (1ULL << GPIO_OUT3) | (1ULL << GPIO_OUT4) | \
                            (1ULL << GPIO_OUT5) | (1ULL << GPIO_OUT6) | (1ULL << GPIO_OUT7) | (1ULL << GPIO_OUT8))

/* -- Types --------------------------------------------------------------------------------------------------------- */
typedef struct
{
   uint32_t u32_VoltageLimit;
   uint32_t u32_KeyCode;
} T_key_adc;

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */
static led_strip_handle_t mt_LedStrip;
static uint32_t mau32_LedColor[STATUS_LED_COUNT];

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * configure the ESP32 IO signals
 */
esp_err_t io_init(void)
{
   esp_err_t t_Err;
   gpio_config_t t_OutGpioCfg =
   {
      .pin_bit_mask = GPIO_OUT_MASK,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = 0,
      .pull_down_en = 1,
      .intr_type = GPIO_INTR_DISABLE
   };
   // RMT Konfiguration
   led_strip_config_t t_LedStripConfig =
   {
      .strip_gpio_num = GPIO_RGB_LED,
      .max_leds = STATUS_LED_COUNT,
      .led_model = LED_MODEL_WS2812,
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
   };
   led_strip_rmt_config_t t_RmtConfig =
   {
      .resolution_hz = 10 * 1000 * 1000, // 10 MHz
   };

   // set all OUTx pins as input with pull-down enabled -> disable all outputs
   t_Err = gpio_config(&t_OutGpioCfg);

   // set optional CAN signals as input with pull-up enabled
   t_OutGpioCfg.pin_bit_mask = ((1ULL << GPIO_CAN_TX) | (1ULL << GPIO_CAN_RX));
   t_OutGpioCfg.pull_up_en = 1u;
   t_OutGpioCfg.pull_down_en = 0u;
   t_Err |= gpio_config(&t_OutGpioCfg);

   // set USENS_EN signal as output and input with pull-down enabled
   t_OutGpioCfg.pin_bit_mask = (1ULL << GPIO_USENS_EN);
   t_OutGpioCfg.mode = GPIO_MODE_INPUT_OUTPUT;
   t_OutGpioCfg.pull_up_en = 0u;
   t_OutGpioCfg.pull_down_en = 1u;
   t_Err |= gpio_config(&t_OutGpioCfg);

   t_Err |= adc_init();

   // configure the RGB LED output
   led_strip_new_rmt_device(&t_LedStripConfig, &t_RmtConfig, &mt_LedStrip);
   led_strip_set_pixel(mt_LedStrip, STATUS_LED_SENSOR, 0u, 0u, 0u);
   led_strip_set_pixel(mt_LedStrip, STATUS_LED_WLAN, 0u, 0u, 0u);
   io_set_led(STATUS_LED_CPU, 0x808000uL);

   if (t_Err == ESP_OK)
   {
      // enable sensor supply voltage
      io_set_sensor_supply_state(1u);
   }
   else
   {
      io_set_led(STATUS_LED_CPU, 0x0000FFuL);
      t_Err = ESP_FAIL;
   }

   return t_Err;
}

/**
 * set sensor supply state: 0 = off,  1 = on
 */
void io_set_sensor_supply_state(const uint32_t ou32_State)
{
   (void)gpio_set_level(GPIO_USENS_EN, ou32_State);
}

/**
 * get sensor supply state: 0 = off,  1 = on
 */
uint32_t io_get_sensor_supply_state(void)
{
   return (uint32_t)gpio_get_level(GPIO_USENS_EN);
}

/**
 * get sensor supply voltage in mV
 */
uint32_t io_get_sensor_supply_voltage(void)
{
   return adc_read_mv(ADC_USENS);
}

/**
 * set status LED color:
 * ou32_Led = LED selector 0..2
 * ou32_Color = LED RGB color value R=[bit0..7], G=[bit8..15], B=[bit16..23]
 */
void io_set_led(const uint32_t ou32_Led, const uint32_t ou32_Color)
{
   if ((mt_LedStrip != NULL) && (ou32_Led < STATUS_LED_COUNT))
   {
      const uint32_t u32_R = ou32_Color & 0xFFuL;
      const uint32_t u32_G = (ou32_Color >>  8u) & 0xFFuL;
      const uint32_t u32_B = (ou32_Color >> 16u) & 0xFFuL;

      led_strip_set_pixel(mt_LedStrip, ou32_Led, u32_R, u32_G, u32_B);
      led_strip_refresh(mt_LedStrip);
      mau32_LedColor[ou32_Led] = ou32_Color;
   }
}

/**
 * get status LED color:
 * ou32_Led = LED selector 0..2
 * u32_Color = LED RGB color value R=[bit0..7], G=[bit8..15], B=[bit16..23]
 */
uint32_t io_get_led_color(const uint32_t ou32_Led)
{
   uint32_t u32_Color = 0uL;
   if (ou32_Led < STATUS_LED_COUNT)
   {
      u32_Color = mau32_LedColor[ou32_Led];
   }

   return u32_Color;
}


/**
 * get status of switch and buttons:
 * state of toggle switch is coded in [bit0..1]: 0=down, 1=midle, 2=up
 * state of KEY1 = bit2
 * state of KEYx: KEY2 = bit3, KEY3 = bit4, KEY4 = bit5
 */
uint32_t io_get_switch_button(void)
{
   static const T_key_adc hat_AdcKey1[5] =
   {
      { 2995u, 0x02u }, { 2490u, 0x05u }, { 1798u, 0x04u }, { 1218u, 0x01u }, {  804u, 0x00u }
   };
   // 3 button keyboard
   static const T_key_adc hat_AdcKeyX[8] =
   {
      { 2600u, 0x20u }, { 2338u, 0x10u }, { 2000u, 0x30u }, { 1725u, 0x08u },
      { 1532u, 0x28u }, { 1367u, 0x18u }, { 1242u, 0x38u }, { 1000u, 0x00u }
   };
   const uint32_t u32_AdcKey1 = adc_read_mv(ADC_KEY1);
   const uint32_t u32_AdcKeyX = adc_read_mv(ADC_KEYX);
   uint32_t u32_Key = 0x06u;
   uint32_t u32_KeyX = 0;
   int i;

   for (i=0; i<5; i++)
   {
      if (u32_AdcKey1 < hat_AdcKey1[i].u32_VoltageLimit)
      {
         u32_Key = hat_AdcKey1[i].u32_KeyCode;
      }
      else
      {
         break;
      }
   }

   for (i=0; i<8; i++)
   {
      if (u32_AdcKeyX < hat_AdcKeyX[i].u32_VoltageLimit)
      {
         u32_KeyX = hat_AdcKeyX[i].u32_KeyCode;
      }
      else
      {
         break;
      }
   }

   return u32_Key | u32_KeyX;
}
