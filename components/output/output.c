/*
   output control

   manage PWM controlled outputs
*/

/* --- Includes ------------------------------------------------------------ */
#include "output.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

/* --- Defines ------------------------------------------------------------- */

#define OUTPUT_PWM_AC_RESOLUTION          20u      // AC type PWM resolution in steps of 10ms. Maximum = 32
#define OUTPUT_PWM_AC_TIMER_INTERVAL_US   10000u   // AC type PWM timer interval in us

#define OUTPUT_PWM_RESOLUTION             (1u << LEDC_TIMER_10_BIT)  // PWM resolution in digits (10bit)
#define OUTPUT_PWM_FREQUENZY_HZ           2000u                      // PWM frequency for PWM type outputs


/* --- Types --------------------------------------------------------------- */

/**
 * @brief output PWM constant defines
 *
 */
typedef struct
{
   gpio_num_t t_OutGpio;            /*!< define the GPIO pin */
   uint16_t u16_PhaseOffsetPwm;     /*!< phase offset for PWM */
   uint16_t u16_PhaseOffsetPwmAc;   /*!< phase offset for PWM AC */
} T_output_def;


/**
 * @brief output PWM constant defines
 *
 */
typedef struct
{
   T_output_def at_OutputDef[OUTPUT_NUM_CHANNEL_MAX];          /*!< define the GPIO pin properties */
   uint32_t au32_PwmAcPattern[OUTPUT_PWM_AC_RESOLUTION+1u];    /*!< PWM (AC) patterns */
} T_output_pwm_def;

/**
 * @brief output value and PWM pattern data
 *
 */
typedef struct
{
   T_out_channel_data t_OutSetup;   /*!< output setup */
   int32_t s32_OutValue;            /*!< output value */
   uint32_t u32_PwmAcPattern;       /*!< PWM (AC) pattern */
   uint8_t u8_LEDC_Chn;             /*!< LEDC driver channel */
} T_output;


/* --- Module global variables --------------------------------------------- */

static const T_output_pwm_def mt_PwmDefs =
{
   .at_OutputDef =
   {
      { GPIO_OUT1,                                     0u,                                        0u },
      { GPIO_OUT2, (uint16_t)(OUTPUT_PWM_RESOLUTION*0.25), (uint16_t)(OUTPUT_PWM_AC_RESOLUTION*0.25) },
      { GPIO_OUT3, (uint16_t)(OUTPUT_PWM_RESOLUTION*0.50), (uint16_t)(OUTPUT_PWM_AC_RESOLUTION*0.50) },
      { GPIO_OUT4, (uint16_t)(OUTPUT_PWM_RESOLUTION*0.75), (uint16_t)(OUTPUT_PWM_AC_RESOLUTION*0.75) },
      { GPIO_OUT5,                                     0u,                                        0u },
      { GPIO_OUT6, (uint16_t)(OUTPUT_PWM_RESOLUTION*0.25), (uint16_t)(OUTPUT_PWM_AC_RESOLUTION*0.25) },
      { GPIO_OUT7, (uint16_t)(OUTPUT_PWM_RESOLUTION*0.50), (uint16_t)(OUTPUT_PWM_AC_RESOLUTION*0.50) },
      { GPIO_OUT8, (uint16_t)(OUTPUT_PWM_RESOLUTION*0.75), (uint16_t)(OUTPUT_PWM_AC_RESOLUTION*0.75) }
   },
   .au32_PwmAcPattern =
   {
      0x00000000uL,
      0x00000001uL, 0x00000003uL, 0x00000007uL, 0x0000000FuL,
      0x0000001FuL, 0x00001C07uL, 0x00001C0FuL, 0x00003C0FuL,
      0x00003C1FuL, 0x00007C1FuL, 0x00007C3FuL, 0x0000FC3FuL,
      0x0000FC7FuL, 0x0001FC7FuL, 0x00007FFFuL, 0x0000FFFFuL,
      0x0001FFFFuL, 0x0003FFFFuL, 0x0007FFFFuL, 0x000FFFFFuL
/*
      0x00000001uL, 0x00000003uL, 0x00000007uL, 0x0000000FuL,
      0x0000001FuL, 0x0000003FuL, 0x0000007FuL, 0x000000FFuL,
      0x000001FFuL, 0x000003FFuL, 0x000007FFuL, 0x00000FFFuL,
      0x00001FFFuL, 0x00003FFFuL, 0x00007FFFuL, 0x0000FFFFuL,
      0x0001FFFFuL, 0x0003FFFFuL, 0x0007FFFFuL, 0x000FFFFFuL
*/
/*
      0x00000001uL, 0x00000401uL, 0x00004081uL, 0x00008421uL,
      0x00011111uL, 0x00024489uL, 0x00049249uL, 0x0004A529uL,
      0x000554A5uL, 0x00055555uL, 0x000AAB5AuL, 0x000B5AD6uL,
      0x000B6DB6uL, 0x000DBB76uL, 0x000EEEEEuL, 0x000F7BDEuL,
      0x000FBF7EuL, 0x000FFBFEuL, 0x000FFFFEuL, 0x000FFFFFuL
*/
   },
};

static T_output mat_Output[OUTPUT_NUM_CHANNEL_MAX];
static uint8_t mu8_PwmAcPhase;
static esp_timer_handle_t mt_pwm_timer;


/* --- Module global prototypes -------------------------------------------- */
static esp_err_t m_output_install_interval_timer(void);
static void m_output_pwm_interval_timer_cb(void* arg);

/* --- Implementation ------------------------------------------------------ */

/**
 * @brief initialize the output module
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - Others: Fail
 */
esp_err_t output_init(void)
{
   esp_err_t err = ESP_OK;
   uint8_t u8_LedcChannel = LEDC_CHANNEL_0;
   gpio_config_t t_OutGpioCfg =
   {
      .pin_bit_mask = 0,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = 0,
      .pull_down_en = 1,
      .intr_type = GPIO_INTR_DISABLE
   };
   static const ledc_timer_config_t ledc_timer =
   {
      .speed_mode       = LEDC_LOW_SPEED_MODE,
      .timer_num        = LEDC_TIMER_0,
      .duty_resolution  = LEDC_TIMER_10_BIT,
      .freq_hz          = OUTPUT_PWM_FREQUENZY_HZ,
      .clk_cfg          = LEDC_AUTO_CLK
   };
   ledc_channel_config_t ledc_channel =
   {
      .gpio_num       = 0,
      .speed_mode     = LEDC_LOW_SPEED_MODE,
      .channel        = 0,
      .intr_type      = LEDC_INTR_DISABLE,
      .timer_sel      = LEDC_TIMER_0,
      .duty           = 0,
      .hpoint         = 0
  };

   // init the LEDC pwm timer
   err = ledc_timer_config(&ledc_timer);

   if (err == ESP_OK)
   {
      // init output PWM data
      mu8_PwmAcPhase = 0u;
      for (uint32_t u32_OutChannel = 0u; u32_OutChannel < OUTPUT_NUM_CHANNEL_MAX; u32_OutChannel++)
      {
         const T_output_def * const pt_OutDef = &mt_PwmDefs.at_OutputDef[u32_OutChannel];
         T_output * const pt_Output = &mat_Output[u32_OutChannel];
         T_out_channel_data * const pt_OutSetup = &pt_Output->t_OutSetup;

         // get output data from NVM
         (void)output_data_load(u32_OutChannel, pt_OutSetup);

         if (pt_OutSetup->u8_OutType == OUTPUT_TYPE_PWM)
         {
            if (u8_LedcChannel < LEDC_CHANNEL_MAX)
            {
               ledc_channel.gpio_num = pt_OutDef->t_OutGpio;
               ledc_channel.channel = u8_LedcChannel;
               ledc_channel.hpoint = pt_OutDef->u16_PhaseOffsetPwm;
               ledc_channel_config(&ledc_channel);
               // store LEDC channel used for this PWM output
               pt_Output->u8_LEDC_Chn = u8_LedcChannel;
               u8_LedcChannel++;
            }
            else
            {
               // no more LEDC channels available: configure output as type digital
               pt_OutSetup->u8_OutType = OUTPUT_TYPE_DIGITAL;
            }
         }

         if ((pt_OutSetup->u8_OutType == OUTPUT_TYPE_DIGITAL) || (pt_OutSetup->u8_OutType == OUTPUT_TYPE_PWM_AC))
         {
            // setup bitmask for all GPIO output pins used as digital output or AC type PWM
            t_OutGpioCfg.pin_bit_mask |= (1ULL << pt_OutDef->t_OutGpio);
         }

         (void)output_set_pwm(u32_OutChannel, 0);
      }

      // init GPIOs as outputs
      err = gpio_config(&t_OutGpioCfg);
   }

   if (err == ESP_OK)
   {
      // install interval timer
      err = m_output_install_interval_timer();
   }

   return err;
}

/**
 * @brief Create and start PWM interval timer
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - Others: Fail
 */
static esp_err_t m_output_install_interval_timer(void)
{
   esp_err_t err;
   const esp_timer_create_args_t pwm_timer_args =
   {
      .callback = &m_output_pwm_interval_timer_cb,
      .arg = (void*)&mt_pwm_timer,
      .dispatch_method = ESP_TIMER_ISR,
      /* name is optional, but may help identify the timer when debugging */
      .name = "PWM_timer"
   };

   err = esp_timer_create(&pwm_timer_args, &mt_pwm_timer);
   if (err == ESP_OK)
   {
      err = esp_timer_start_periodic(mt_pwm_timer, OUTPUT_PWM_AC_TIMER_INTERVAL_US);
   }

   return err;
}


/**
 * @brief read output setup data
 *
 * @param ou32_Channel  output channel, range: [0 .. OUTPUT_NUM_CHANNEL_MAX-1]
 * @param opt_OutSetup  return poutput setup data
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: ou32_Channel out of range
 */
esp_err_t output_get_setup(const uint32_t ou32_Channel, T_out_channel_data * const opt_OutSetup)
{
   esp_err_t err = ESP_ERR_INVALID_ARG;

   if (ou32_Channel < OUTPUT_NUM_CHANNEL_MAX)
   {
      *opt_OutSetup = mat_Output[ou32_Channel].t_OutSetup;
      err = ESP_OK;
   }

   return err;
}

/**
 * @brief Set output PWM value
 *
 * @param ou32_Channel  output channel, range: [0 .. OUTPUT_NUM_CHANNEL_MAX-1]
 * @param os32_OutPwm   output PWM value, range [OUTPUT_PWM_MIN .. OUTPUT_PWM_MAX]
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: ou32_Channel or os32_OutPwm out of range
 *  - Others: Fail
 */
esp_err_t output_set_pwm(const uint32_t ou32_Channel, const int32_t os32_OutPwm)
{
   esp_err_t err = ESP_ERR_INVALID_ARG;

   if ((ou32_Channel < OUTPUT_NUM_CHANNEL_MAX) && (os32_OutPwm >= OUTPUT_PWM_MIN) && (os32_OutPwm <= OUTPUT_PWM_MAX))
   {
      T_output * const pt_Output = &mat_Output[ou32_Channel];
      const uint8_t u8_OutType = pt_Output->t_OutSetup.u8_OutType;

      pt_Output->s32_OutValue = os32_OutPwm;

      if (u8_OutType == OUTPUT_TYPE_DIGITAL)
      {
         const T_output_def * const pt_OutDef = &mt_PwmDefs.at_OutputDef[ou32_Channel];
         const uint32_t u32_OutLevel = (os32_OutPwm > (OUTPUT_PWM_MAX/2)) ? 1uL : 0uL;
         gpio_set_level(pt_OutDef->t_OutGpio, u32_OutLevel);
      }
      else if (u8_OutType == OUTPUT_TYPE_PWM)
      {
         const uint32_t u32_ScaledPwmDuty = (uint32_t)(((os32_OutPwm * OUTPUT_PWM_RESOLUTION) + (OUTPUT_PWM_MAX/2)) / OUTPUT_PWM_MAX);
         ledc_set_duty(LEDC_LOW_SPEED_MODE, pt_Output->u8_LEDC_Chn, u32_ScaledPwmDuty);
         ledc_update_duty(LEDC_LOW_SPEED_MODE, pt_Output->u8_LEDC_Chn);
      }
      else if (u8_OutType == OUTPUT_TYPE_PWM_AC)
      {
         const uint32_t u32_ScaledPwm = (uint32_t)(((os32_OutPwm * OUTPUT_PWM_AC_RESOLUTION) + (OUTPUT_PWM_MAX/2)) / OUTPUT_PWM_MAX);
         pt_Output->u32_PwmAcPattern = mt_PwmDefs.au32_PwmAcPattern[u32_ScaledPwm];
      }
      else
      {
         // OUTPUT_TYPE_INVALID: output value is always zero
         pt_Output->s32_OutValue = 0;
      }

      err = ESP_OK;
   }

   return err;
}

/**
 * @brief Get output PWM value
 *
 * @param ou32_Channel  output channel, range: [0 .. OUTPUT_NUM_CHANNEL_MAX-1]
 * @param ops32_OutPwm  reference, returns the last set PWM value
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: ou32_Channel out of range
 *  - Others: Fail
 */
esp_err_t output_get_pwm(const uint32_t ou32_Channel, int32_t * const ops32_OutPwm)
{
   esp_err_t err = ESP_ERR_INVALID_ARG;

   if (ou32_Channel < OUTPUT_NUM_CHANNEL_MAX)
   {
      *ops32_OutPwm = mat_Output[ou32_Channel].s32_OutValue;

      err = ESP_OK;
   }

   return err;
}

/**
 * @brief output PWM interval timer
 *
 * @param arg  timer handle
 */
static void m_output_pwm_interval_timer_cb(void* arg)
{
   (void)arg;

   for (uint32_t u32_Channel = 0u; u32_Channel < OUTPUT_NUM_CHANNEL_MAX; u32_Channel++)
   {
      T_output * const pt_Output = &mat_Output[u32_Channel];

      if (pt_Output->t_OutSetup.u8_OutType == OUTPUT_TYPE_PWM_AC)
      {
         const T_output_def * const pt_OutDef = &mt_PwmDefs.at_OutputDef[u32_Channel];
         uint32_t u32_OutLevel;
         uint32_t u32_PwmPhaseIdx = mu8_PwmAcPhase + (uint32_t)pt_OutDef->u16_PhaseOffsetPwmAc;

         if (u32_PwmPhaseIdx >= OUTPUT_PWM_AC_RESOLUTION)
         {
            u32_PwmPhaseIdx -= OUTPUT_PWM_AC_RESOLUTION;
         }
         u32_OutLevel = ((pt_Output->u32_PwmAcPattern & (1uL << u32_PwmPhaseIdx)) != 0u) ? 1uL : 0uL;

         gpio_set_level(pt_OutDef->t_OutGpio, u32_OutLevel);
      }
   }

   // increment PWM phase index
   mu8_PwmAcPhase++;
   if (mu8_PwmAcPhase >= OUTPUT_PWM_AC_RESOLUTION)
   {
      mu8_PwmAcPhase = 0u;
   }
}
