//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor_data.h
   \brief provide nvm data for all available sensors
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "sensor_api.h"
#include "bme280_interface.h"
#include "bme68x_interface.h"
#include "scd4x_interface.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

esp_err_t sensor_data_load(void);
esp_err_t sensor_data_store(void);
T_sensor_nvm_data * sensor_data_get(const uint16_t ou16_SensorIdx);
esp_err_t sensor_data_set_name(const uint16_t ou16_SensorIdx, const char * const oac_SensorName);
esp_err_t sensor_data_set_flags(const uint16_t ou16_SensorIdx, const uint8_t ou8_SensorFlags);

#endif /* SENSOR_DATA_H */
