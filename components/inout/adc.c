/*
   ADC control

   reads ADC signals
*/
#include "adc.h"
#include "esp_adc/adc_oneshot.h"


/* --- Defines ------------------------------------------------------------- */

#define NO_OF_SAMPLES      4     // Multisampling


/* --- Types --------------------------------------------------------------- */

typedef struct
{
   adc_channel_t t_AdcChannel;
   uint32_t u32_Scale;
} T_adc_channel;


/* --- Module global variables --------------------------------------------- */

static adc_oneshot_unit_handle_t mt_adc1_handle;
static adc_cali_handle_t mt_adc1_cali_handle;

static const T_adc_channel mat_adc_channel[ADC_CH_MAX] =
{
   [ADC_KEY1]  = { .t_AdcChannel = ADC_CHANNEL_0, .u32_Scale = 1u },
   [ADC_KEYX]  = { .t_AdcChannel = ADC_CHANNEL_3, .u32_Scale = 1u },
   [ADC_USENS] = { .t_AdcChannel = ADC_CHANNEL_6, .u32_Scale = 2u }
};

/* --- Module function prototypes ------------------------------------------ */

/* --- Implementation ------------------------------------------------------ */

/**
 * \brief   initialize the ADC peripheral (ADC1) and all used ADC channels
 *****************************************************************************/
esp_err_t adc_init(void)
{
   static const adc_oneshot_unit_init_cfg_t t_adc_cfg1 =
   {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
   };
   static const adc_oneshot_chan_cfg_t t_adc_chn_cfg =
   {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
   };
   static const adc_cali_line_fitting_config_t t_adc_cali_cfg =
   {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
   };

   esp_err_t ret = adc_oneshot_new_unit(&t_adc_cfg1, &mt_adc1_handle);

   if (ret == ESP_OK)
   {
      for (uint32_t u32_AdcChn = 0u; u32_AdcChn < ADC_CH_MAX; u32_AdcChn++)
      {
         const T_adc_channel* const pt_AdcChn = &mat_adc_channel[u32_AdcChn];
         ret |= adc_oneshot_config_channel(mt_adc1_handle, pt_AdcChn->t_AdcChannel, &t_adc_chn_cfg);
      }
   }

   if (ret == ESP_OK)
   {
      ret = adc_cali_create_scheme_line_fitting(&t_adc_cali_cfg, &mt_adc1_cali_handle);
   }

   if (ret != ESP_OK)
   {
      mt_adc1_handle = NULL;
      mt_adc1_cali_handle = NULL;
      ret = ESP_FAIL;
   }

   return ret;
}


/**
 * \brief   read ADC channel voltage scaled in milli Volt
 *****************************************************************************/
uint32_t adc_read_mv(const uint32_t adc_channel)
{
   const T_adc_channel* const pt_AdcChn = &mat_adc_channel[adc_channel];
   uint32_t u32_mV = 0u;

   if (mt_adc1_handle != NULL)
   {
      int AdcReading = 0;
      int Voltage;

      // Multisampling
      for (int i = 0; i < NO_OF_SAMPLES; i++)
      {
         int AdcRaw;
         if (adc_oneshot_read(mt_adc1_handle, pt_AdcChn->t_AdcChannel, &AdcRaw) == ESP_OK)
         {
            AdcReading += AdcRaw;
         }
      }
      AdcReading /= NO_OF_SAMPLES;

      // convert adc_reading to voltage in mV
      if (adc_cali_raw_to_voltage(mt_adc1_cali_handle, AdcReading, &Voltage) == ESP_OK)
      {
         if (Voltage < 0)
         {
            Voltage = 0;
         }
         if (Voltage > 3000)
         {
            Voltage = 3000;   // limit voltage to 3000mV max
         }
         u32_mV = (uint32_t)Voltage;
      }
   }

   return u32_mV * pt_AdcChn->u32_Scale;
}
