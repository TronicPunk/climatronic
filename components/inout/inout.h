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
#define STATUS_LED_SENSOR        0u             // index SENSOR LED
#define STATUS_LED_WLAN          1u             // index WLAN LED
#define STATUS_LED_CPU           2u             // index CPU LED
#define STATUS_LED_COUNT         3u             // number of LEDs

#define STATUS_LED_RED_MSK       0x00FF0000uL   // red color mask
#define STATUS_LED_RED_BITPOS    16u            // red color bit position
#define STATUS_LED_GREEN_MSK     0x0000FF00uL   // green color mask
#define STATUS_LED_GREEN_BITPOS  8u             // green color bit position
#define STATUS_LED_BLUE_MSK      0x000000FFuL   // blue color mask
#define STATUS_LED_BLUE_BITPOS   0u             // blue color bit position

#define SWITCH_MASK     0x03u    // toggle switch: state 0, 1, 2
#define SWITCH_DOWN     0x00u    // toggle switch: state 0 = down position
#define SWITCH_MID      0x01u    // toggle switch: state 1 = midle position
#define SWITCH_UP       0x02u    // toggle switch: state 2 = up position

#define BUTTON_1_MASK   0x04u    // bit when push button_1 is pressed (LCD KEY1)
#define BUTTON_2_MASK   0x08u    // bit when push button_2 is pressed (LCD KEYx_2)
#define BUTTON_3_MASK   0x10u    // bit when push button_3 is pressed (LCD KEYx_3)
#define BUTTON_4_MASK   0x20u    // bit when push button_4 is pressed (LCD KEYx_4)
#define BUTTON_5_MASK   0x40u    // bit when push button_5 is pressed (LCD KEYx_5)

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

esp_err_t io_init(void);
void     io_set_sensor_supply_state(const uint32_t ou32_State);   // set sensor supply on/off
uint32_t io_get_sensor_supply_state(void);                        // get sensor supply state
uint32_t io_get_sensor_supply_voltage(void);                      // get sensor supply voltage in mV
void     io_set_led_off(void);                                             // set all LEDs off
void     io_set_led(const uint32_t ou32_Led, const uint32_t ou32_Color);   // set LED color
uint32_t io_get_led_color(const uint32_t ou32_Led);                        // get LED color
uint32_t io_get_switch_button(void);                                       // get switch and push button state

/* -- Implementation ------------------------------------------------------------------------------------------------ */

#endif /* INOUT_H */
