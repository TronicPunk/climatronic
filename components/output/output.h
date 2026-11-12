/*
   output control

   manage PWM controlled outputs
*/

#ifndef OUTPUT_H
#define OUTPUT_H

/* --- Includes ------------------------------------------------------------ */
#include "output_data.h"

/* --- Defines ------------------------------------------------------------- */

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
 * @brief read output setup data
 *
 * @param ou32_Channel  output channel, range: [0 .. OUTPUT_NUM_CHANNEL_MAX-1]
 * @param opt_OutSetup  return poutput setup data
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: ou32_Channel out of range
 */
esp_err_t output_get_setup(const uint32_t ou32_Channel, T_out_channel_data * const opt_OutSetup);
esp_err_t output_set_setup(const uint32_t ou32_Channel, const T_out_channel_data * const opt_OutSetup);

/**
 * @brief Set output PWM value
 *
 *        The PWM value is ignored by the output task when
 *        output mode is OUTPUT_OFF or OUTPUT_FIX or the output is not writeable
 *
 * @param ou32_Channel  output channel, range: [0 .. OUT_NUM_CHANNEL_MAX-1]
 * @param os32_OutPwm   output PWM value, range [OUT_PWM_MIN .. OUT_PWM_MAX]
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: ou32_Channel or os32_OutPwm out of range
 *  - Others: Fail
 */
esp_err_t output_set_value(const uint32_t ou32_Channel, const int32_t os32_OutPwm);

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
esp_err_t output_get_value(const uint32_t ou32_Channel, int32_t * const ops32_OutPwm);

#endif // OUTPUT_H