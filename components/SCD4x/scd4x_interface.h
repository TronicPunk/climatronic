//----------------------------------------------------------------------------------------------------------------------
/*!
   \file    scd4x_interface.h
   \brief   SCD40/SCD41 interface: Glue layer between Sensirion Sensor API and ESP32 driver library
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef SCD4X_INTERFACE_H
#define SCD4X_INTERFACE_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <stdio.h>
#include "sensor_api.h"
#include "scd4x_i2c.h"
#include "sensirion_i2c_hal.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */
extern const T_sensor_type gt_SCD41_type;   // delivers sensor type information
extern const T_sensor_api gt_SCD4x_api;     // delivers common sensor API

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

void __force_link_scd4x(void);
esp_err_t scd4x_sensor_select_device(const T_sensor_def * const opt_SensorDef);

#endif /* SCD4X_INTERFACE_H */
