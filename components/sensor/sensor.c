//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor.c
   \brief provide functions for sensor communication
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "sensor_data.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * return number of registered sensors
 */
uint16_t sensor_get_count(void)
{
   return sensor_data_get_count();
}

/**
 * return sensor information
 */
esp_err_t sensor_get_info(const uint16_t ou16_SensorIdx, const T_sensor_info ** opt_SensorInfo)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const T_sensor_def* const pt_SensorDef = sensor_data_get_handle(ou16_SensorIdx);

   if (pt_SensorDef != NULL)
   {
      *opt_SensorInfo = &pt_SensorDef->t_SensorInfo;
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * call sensor init on specified IIC bus
 */
esp_err_t sensor_init(const uint16_t ou16_SensorIdx)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const T_sensor_def* const pt_SensorDef = sensor_data_get_handle(ou16_SensorIdx);

   if (pt_SensorDef != NULL)
   {
      t_Err = pt_SensorDef->pt_SensorApi->pr_SensorInit(pt_SensorDef);
   }

   return t_Err;
}

/**
 * call sensor process data (the sensor data processing task)
 */
esp_err_t sensor_process_data(const uint16_t ou16_SensorIdx)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const T_sensor_def* const pt_SensorDef = sensor_data_get_handle(ou16_SensorIdx);

   if (pt_SensorDef != NULL)
   {
      // sensor initialized?
      if (pt_SensorDef->pt_SensorInstance->pv_Handle != NULL)
      {
         t_Err = pt_SensorDef->pt_SensorApi->pr_SensorProcessData(pt_SensorDef);
      }
      else
      {
         t_Err = ESP_ERR_INVALID_STATE;
      }
   }

   return t_Err;
}

/**
 * read sensor data
 */
esp_err_t sensor_get_data(uint16_t ou16_SensorIdx, T_sensor_data * opt_SensorData)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const T_sensor_def* const pt_SensorDef = sensor_data_get_handle(ou16_SensorIdx);

   if (pt_SensorDef != NULL)
   {
      // sensor initialized?
      if (pt_SensorDef->pt_SensorInstance->pv_Handle != NULL)
      {
         t_Err = pt_SensorDef->pt_SensorApi->pr_SensorGetData(pt_SensorDef, opt_SensorData);
      }
      else
      {
         t_Err = ESP_ERR_INVALID_STATE;
      }
   }

   return t_Err;
}

/**
 * set all sensor values invalid
 */
void sensor_set_data_invalid(T_sensor_data * const opt_SensorData)
{
   opt_SensorData->f32_Temperature = SENSOR_VALUE_INVALID;
   opt_SensorData->f32_Pressure = SENSOR_VALUE_INVALID;
   opt_SensorData->f32_Humidity = SENSOR_VALUE_INVALID;
   opt_SensorData->f32_CO2 = SENSOR_VALUE_INVALID;
   opt_SensorData->f32_IAQ = SENSOR_VALUE_INVALID;
   opt_SensorData->u32_AgeSec = SENSOR_TIMESTAMP_INVALID;
}
