/*
   output control

   manage PWM controlled outputs
*/

/* --- Includes ------------------------------------------------------------ */
#include "esp_err.h"
#include "esp_types.h"

/* --- Defines ------------------------------------------------------------- */

#define OUTPUT_NUM_CHANNEL_MAX   (0x8uL)  // number of output channels
#define OUTPUT_PWM_MIN           (0)      // min. PWM value (0%)
#define OUTPUT_PWM_MAX           (1000)   // max. PWM value (100%)


/* --- Types --------------------------------------------------------------- */

/* --- Prototypes ---------------------------------------------------------- */

/**
 * @brief initialize the output module
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - Others: Fail
 */
esp_err_t output_init(void);

/**
 * @brief Set output PWM value
 *
 * @param ou32_Channel  output channel, range: [0 .. OUT_NUM_CHANNEL_MAX-1]
 * @param os32_OutPwm   output PWM value, range [OUT_PWM_MIN .. OUT_PWM_MAX]
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: ou32_Channel or os32_OutPwm out of range
 *  - Others: Fail
 */
esp_err_t output_set_pwm(const uint32_t ou32_Channel, const int32_t os32_OutPwm);

/**
 * @brief Get output PWM value
 *
 * @param ou32_Channel  output channel, range: [0 .. OUT_NUM_CHANNEL_MAX-1]
 * @param ops32_OutPwm  reference, returns the last set PWM value
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: ou32_Channel out of range
 *  - Others: Fail
 */
esp_err_t output_get_pwm(const uint32_t ou32_Channel, int32_t * const ops32_OutPwm);
