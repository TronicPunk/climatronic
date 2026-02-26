//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor.c
   \brief provide functions for sensor communication
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "sensor_data.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define SENSOR_MEASUREMENT_TASK_CYCLE_MS  1000u    // seons measurement task cycle time in ms

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */
static void m_sensor_task(TimerHandle_t pxTimer);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */
static StaticTimer_t mt_TimerBuffer;
static TimerHandle_t mt_SensorTimer;

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * The cyclic sensor measurement task function
 *
 */
static void m_sensor_task(TimerHandle_t pxTimer)
{
   uint16_t u16_SensorCnt = sensor_data_get_count();

   // process all connected sensors
   for (uint16_t u16_SensorIdx = 0; u16_SensorIdx < u16_SensorCnt; u16_SensorIdx++)
   {
      (void)sensor_process_data(u16_SensorIdx);    // trigger sensor measurement cycle and data processing
   }
}

/**
 * init the sensor measurement process
 */
esp_err_t sensor_init_task(void)
{
   esp_err_t t_Err = sensor_detect();

   if (t_Err == ESP_OK)
   {
      t_Err = ESP_FAIL;

      mt_SensorTimer = xTimerCreateStatic("SensorTask", SENSOR_MEASUREMENT_TASK_CYCLE_MS/portTICK_PERIOD_MS, pdTRUE,
                                          NULL, &m_sensor_task, &mt_TimerBuffer);
      if (mt_SensorTimer != NULL)
      {
         if (xTimerStart(mt_SensorTimer, 0u) == pdPASS)
         {
            t_Err = ESP_OK;
         }
      }
   }

   return t_Err;
}

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
esp_err_t sensor_get_info(const uint16_t ou16_SensorIdx, T_sensor_info * const opt_SensorInfo)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const T_sensor_instance * const pt_Sensor = sensor_data_get_handle(ou16_SensorIdx);

   if (pt_Sensor != NULL)
   {
      opt_SensorInfo->pt_Sensor = pt_Sensor->pt_SensorDescriptor->pt_Sensor;
      opt_SensorInfo->s_Name = pt_Sensor->s_SensorName;
      opt_SensorInfo->u32_IicFreq = pt_Sensor->u32_IicFreq;
      opt_SensorInfo->u8_IicBus = pt_Sensor->u8_IicBus;
      opt_SensorInfo->u8_IicAddr = pt_Sensor->u8_IicAddr;
      t_Err = ESP_OK;
   }

   return t_Err;
}

/**
 * try tro detect any installed sensors
 */
esp_err_t sensor_detect(void)
{
   esp_err_t t_Err = ESP_FAIL;

   for (uint32_t u32_IicBusIdx = 0u; u32_IicBusIdx < ICC_BUS_COUNT; u32_IicBusIdx++)
   {
      // probe for all possible sensors at this IIC bus
      for (uint32_t u32_SensorTypeIdx = 0u; u32_SensorTypeIdx < SENSOR_TYPE_COUNT; u32_SensorTypeIdx++)
      {
         const T_sensor_bus_init * pt_IicBusInit = &gat_SensorbusInit[u32_IicBusIdx];
         const T_sensor_descriptor * pt_SensorDescriptor = gapt_SensorTypes[u32_SensorTypeIdx]; // 0 = BME280, 1 = BME680, 2 = SCD41
         const uint8_t u8_IicAddr = pt_SensorDescriptor->au8_IicAddr[0];
         T_sensor_instance * pt_NewSensor = NULL;
         esp_err_t t_SensorFound;

         t_SensorFound = pt_SensorDescriptor->pt_SensorApi->pr_SensorInit(pt_IicBusInit->u8_IicBus, u8_IicAddr, pt_IicBusInit->u32_IicFreqHz, &pt_NewSensor);
         if (t_SensorFound == ESP_OK)
         {
            sensor_data_register_sensor(pt_NewSensor);
            t_Err = ESP_OK;
         }
      }
   }

   return t_Err;
}

/**
 * call sensor process data (the sensor data processing task)
 */
esp_err_t sensor_process_data(const uint16_t ou16_SensorIdx)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   T_sensor_instance * const pt_Sensor = sensor_data_get_handle(ou16_SensorIdx);

   if (pt_Sensor != NULL)
   {
      // sensor initialized?
      if (pt_Sensor->pv_Handle != NULL)
      {
         t_Err = pt_Sensor->pt_SensorDescriptor->pt_SensorApi->pr_SensorProcessData(pt_Sensor);
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
esp_err_t sensor_get_data(uint16_t ou16_SensorIdx, T_sensor_data * const opt_SensorData)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   T_sensor_instance * const pt_Sensor = sensor_data_get_handle(ou16_SensorIdx);

   if (pt_Sensor != NULL)
   {
      // sensor initialized?
      if (pt_Sensor->pv_Handle != NULL)
      {
         t_Err = pt_Sensor->pt_SensorDescriptor->pt_SensorApi->pr_SensorGetData(pt_Sensor, opt_SensorData);
      }
      else
      {
         t_Err = ESP_ERR_INVALID_STATE;
      }
   }

   return t_Err;
}

/**
 * send sensor command
 */
esp_err_t sensor_command(const uint16_t ou16_SensorIdx, const uint16_t ou16_Command, uint32_t * const  opu32_Parameter)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   T_sensor_instance * const pt_Sensor = sensor_data_get_handle(ou16_SensorIdx);

   if (pt_Sensor != NULL)
   {
      // sensor initialized?
      if (pt_Sensor->pv_Handle != NULL)
      {
         t_Err = pt_Sensor->pt_SensorDescriptor->pt_SensorApi->pr_SensorCommand(pt_Sensor, ou16_Command, opu32_Parameter);
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
   opt_SensorData->u32_TimeStampSec = SENSOR_TIMESTAMP_INVALID;
}
