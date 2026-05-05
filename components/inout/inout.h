//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  inout.h
   \brief provide low level input/output functions
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef INOUT_H
#define INOUT_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "esp_types.h"
#include "esp_err.h"
#include "adc.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define STATUS_LED_SENSOR  0u
#define STATUS_LED_WLAN    1u
#define STATUS_LED_CPU     2u
#define STATUS_LED_COUNT   3u

#define SWITCH_MASK        0x03u    // toggle switch: state 0, 1, 2
#define BUTTON_1_MASK      0x04u    // bit when push button_1 is pressed (LCD KEY1)
#define BUTTON_2_MASK      0x08u    // bit when push button_2 is pressed (LCD KEYx_2)
#define BUTTON_3_MASK      0x10u    // bit when push button_3 is pressed (LCD KEYx_3)
#define BUTTON_4_MASK      0x20u    // bit when push button_4 is pressed (LCD KEYx_4)
#define BUTTON_5_MASK      0x40u    // bit when push button_5 is pressed (LCD KEYx_5)

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

esp_err_t io_init(void);
void     io_set_sensor_supply_state(const uint32_t ou32_State);   // set sensor supply on/off
uint32_t io_get_sensor_supply_state(void);                        // get sensor supply state
uint32_t io_get_sensor_supply_voltage(void);                      // get sensor supply voltage in mV
void     io_set_led(const uint32_t ou32_Led, const uint32_t ou32_Color);   // set status LED color
uint32_t io_get_led_color(const uint32_t ou32_Led);                        // get status LED color
uint32_t io_get_switch_button(void);                              // get switch and push button state

/* -- Implementation ------------------------------------------------------------------------------------------------ */

#endif /* INOUT_H */
