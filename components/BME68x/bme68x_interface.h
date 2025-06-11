//----------------------------------------------------------------------------------------------------------------------
/*!
   \file    bme68x_interface.h
   \brief   BME68x interface: Glue layer between BOSCH Sensor API and ESP32 driver library
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef BME68X_INTERFACE_H
#define BME68X_INTERFACE_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <stdio.h>
#include "sensor_api.h"
#include "bme68x.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */
extern const T_sensor_type gt_BME680_type;   // delivers sensor type information
extern const T_sensor_api gt_BME68x_api;     // delivers common sensor API

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

extern void __force_link_bme68x(void);

#endif /* BME68X_INTERFACE_H */
