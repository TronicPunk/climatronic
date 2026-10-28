//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor_name.h
   \brief provide functions for sensor naming
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef SENSOR_NAME_H
#define SENSOR_NAME_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "sensor_api.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

#define SENSOR_NAME_LEN_MAX      24 // max. length of sensor name string (without \0)

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

esp_err_t sensor_name_load(void);
esp_err_t sensor_name_store(void);
char * sensor_name_get(const T_sensor_instance * opt_SensorInstance);
esp_err_t sensor_name_set(const T_sensor_instance * opt_SensorInstance, const char * const oac_SensorName);
char * sensor_name_get_idx(const uint16_t ou16_SensorIdx);
esp_err_t sensor_name_set_idx(const uint16_t ou16_SensorIdx, const char * const oac_SensorName);

#endif /* SENSOR_NAME_H */
