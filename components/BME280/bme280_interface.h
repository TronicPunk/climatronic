//----------------------------------------------------------------------------------------------------------------------
/*!
   \file    bme280_interface.h
   \brief   BME280 interface: Glue layer between BOSCH Sensor API and ESP32 driver library
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef BME280_INTERFACE_H
#define BME280_INTERFACE_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <stdio.h>
#include "sensor_api.h"
#include "bme280.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

#define BME280_WRITE_REGS_MAX    (10u)    /* Typically not to write more than 10 registers */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */
extern const T_sensor_descriptor gt_BME280_descriptor;   // delivers the sensor description

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

extern void __force_link_bme280(void);

#endif /* BME280_INTERFACE_H */
