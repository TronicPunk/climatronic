//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor_api.h
   \brief provide definition of generic internal sensor API
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef SENSOR_API_H
#define SENSOR_API_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "sensor.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

typedef struct _sensor_instance
{
   uint64_t u64_TimeStampUs;        // timestamp of last sensor data sample (absolute system time in usec)
   uint32_t u32_NextCallUs;         // time until next data processing call (relative in usec)
   T_sensor_data t_SensorData;      // sensor data sample
   void * pv_Handle;                // sensor handle with driver specific information
} T_sensor_instance;

typedef struct _sensor_def
{
   const T_sensor_info t_SensorInfo;
   const struct _sensor_api * const pt_SensorApi;
   const uint32_t u32_UpdateRateUs;
   T_sensor_instance * const pt_SensorInstance;
} T_sensor_def;

typedef struct _sensor_api
{
   esp_err_t (*pr_SensorInit)(const T_sensor_def * const opt_SensorDef);
   esp_err_t (*pr_SensorProcessData)(const T_sensor_def * const opt_SensorDef);
   esp_err_t (*pr_SensorGetData)(const T_sensor_def * const opt_SensorDef, T_sensor_data * const opt_SensorData);
} T_sensor_api;

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

#endif /* SENSOR_API_H */
