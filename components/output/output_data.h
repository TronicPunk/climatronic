/*
   output configuration and setup data
   load/store output data from/to NVM
*/

/* --- Includes ------------------------------------------------------------ */
#include "esp_types.h"
#include "esp_err.h"

/* --- Defines ------------------------------------------------------------- */

#define OUTPUT_NUM_CHANNEL_MAX   8u             // number of output channels
#define OUTPUT_NAME_MAX          24             // max. length of output channel name string (without \0)
#define OUTPUT_PWM_MIN           0              // min. PWM value (0%)
#define OUTPUT_PWM_MAX           1000           // max. PWM value (100%)

#define OUTPUT_TYPE_INVALID      0u             // output type invalid, always off
#define OUTPUT_TYPE_DIGITAL      1u             // digital output type, can be switch on/off
#define OUTPUT_TYPE_PWM          2u             // PWM type (high speed PWM signal)
#define OUTPUT_TYPE_PWM_AC       3u             // PWM type AC (AC half-wave PWM control)
#define OUTPUT_TYPE_MAX          4u             // number of output types

#define GPIO_OUT1                GPIO_NUM_32    // GPIO for OUT_1
#define GPIO_OUT2                GPIO_NUM_33    // GPIO for OUT_2
#define GPIO_OUT3                GPIO_NUM_25    // GPIO for OUT_3
#define GPIO_OUT4                GPIO_NUM_26    // GPIO for OUT_4
#define GPIO_OUT5                GPIO_NUM_27    // GPIO for OUT_5
#define GPIO_OUT6                GPIO_NUM_13    // GPIO for OUT_6
#define GPIO_OUT7                GPIO_NUM_16    // GPIO for OUT_7
#define GPIO_OUT8                GPIO_NUM_17    // GPIO for OUT_8

/* --- Types --------------------------------------------------------------- */

// output data set
typedef struct
{
   uint16_t u16_OutValueDef;                 // output default value
   uint8_t  u8_OutType;                      // output type
   char     ac_OutName[OUTPUT_NAME_MAX+1];   // output name string (max. 24 char + \0)
} T_out_channel_data;

/* --- Prototypes ---------------------------------------------------------- */

esp_err_t output_data_load(const uint32_t ou32_Channel, T_out_channel_data * const opt_OutChannelSetup);
esp_err_t output_data_store(const uint32_t ou32_Channel, const T_out_channel_data * const opt_OutChannelSetup);
