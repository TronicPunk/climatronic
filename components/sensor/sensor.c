//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor.c
   \brief provide functions for sensor communication
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "sensor_data.h"
#include "system_data.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

#define SENSOR_TYPE_COUNT                 3u       // number of different supported IIC sensors
// note: SENSOR_COUNT_MAX must be <= SENSOR_NVM_DATA_MAX to guarantee data storage for each sensor
#define SENSOR_COUNT_MAX                  16u      // max. number of connected sensors
#define SENSOR_MEASUREMENT_TASK_CYCLE_MS  1000u    // sensor measurement task cycle time in ms

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */
static void m_sensor_task(TimerHandle_t pxTimer);
static esp_err_t m_sensor_detect(void);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static uint16_t mu16_SensorCount = 0u;  // number of installed (detected) sensors
static T_sensor_instance * mapt_SensorList[SENSOR_COUNT_MAX];
static StaticTimer_t mt_TimerBuffer;
static TimerHandle_t mt_SensorTimer;

static const T_sensor_descriptor * mapt_SensorTypes[SENSOR_TYPE_COUNT] =
{
   &gt_BME280_descriptor,  // this sensor driver supports BME280 and also BMP280 (without humidity measurement)
   &gt_BME680_descriptor,
   &gt_SCD41_descriptor,
};


/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * The cyclic sensor measurement task function
 *
 */
static void m_sensor_task(TimerHandle_t pxTimer)
{
   uint16_t u16_SensorCnt = sensor_get_count();

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
   esp_err_t t_Err;

   (void)sensor_data_load();
   t_Err = m_sensor_detect();

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
 * try to detect any installed sensors
 */
static esp_err_t m_sensor_detect(void)
{
   esp_err_t t_Err = ESP_FAIL;

   // clear sensor list
   for (uint32_t u32_Sensor = 0u; u32_Sensor < SENSOR_COUNT_MAX; u32_Sensor++)
   {
      mapt_SensorList[u32_Sensor] = NULL;
   }
   mu16_SensorCount = 0;

   for (uint32_t u32_IicBus = 0u; u32_IicBus < I2C_BUS_COUNT; u32_IicBus++)
   {
      uint8_t u8_I2cFreq;
      (void)system_data_get_i2c_freq(u32_IicBus, &u8_I2cFreq);
      const uint32_t u32_IicFreqHz = (u8_I2cFreq == I2C_BUS_FREQ_400K) ? 400000u : 100000u;

      // probe for all possible sensors at this IIC bus
      // u32_SensorTypeIdx 0 = BME280/BMP280, 1 = BME680, 2 = SCD41
      for (uint32_t u32_SensorTypeIdx = 0u; u32_SensorTypeIdx < SENSOR_TYPE_COUNT; u32_SensorTypeIdx++)
      {
         const T_sensor_descriptor * pt_SensorDescriptor = mapt_SensorTypes[u32_SensorTypeIdx];
         const uint8_t u8_IicAddr = pt_SensorDescriptor->au8_IicAddr[0];
         T_sensor_instance * pt_NewSensor = NULL;
         esp_err_t t_SensorFound;

         t_SensorFound = pt_SensorDescriptor->pt_SensorApi->pr_SensorInit(u32_IicBus, u8_IicAddr, u32_IicFreqHz, &pt_NewSensor);
         if (t_SensorFound == ESP_OK)
         {
            mapt_SensorList[mu16_SensorCount] = pt_NewSensor;
            mu16_SensorCount++;
            t_Err = ESP_OK;

            if (mu16_SensorCount >= SENSOR_COUNT_MAX)
            {
               u32_IicBus = I2C_BUS_COUNT;
               u32_SensorTypeIdx = SENSOR_TYPE_COUNT;
               break;
            }
         }
      }
   }

   if (t_Err == ESP_OK)
   {
      // do sensor naming
      for (uint16_t u16_SensorIdx = 0; u16_SensorIdx < mu16_SensorCount; u16_SensorIdx++)
      {
         T_sensor_instance * const pt_Sensor = sensor_get_handle(u16_SensorIdx);
         pt_Sensor->pt_SensorNvmData = sensor_data_get(u16_SensorIdx);
      }
      t_Err = sensor_data_store();
   }

   return t_Err;
}

/**
 * return number of registered sensors
 */
uint16_t sensor_get_count(void)
{
   return mu16_SensorCount;
}

/**
 * Return sensor handle. Return NULL if ou16_SensorIdx is out of range
 */
T_sensor_instance* sensor_get_handle(const uint16_t ou16_SensorIdx)
{
   T_sensor_instance* pt_Sensor = NULL;

   if (ou16_SensorIdx < mu16_SensorCount)
   {
      pt_Sensor = mapt_SensorList[ou16_SensorIdx];
   }

   return pt_Sensor;
}

/**
 * return sensor information
 */
esp_err_t sensor_get_info(const uint16_t ou16_SensorIdx, T_sensor_info * const opt_SensorInfo)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const T_sensor_instance * const pt_Sensor = sensor_get_handle(ou16_SensorIdx);

   if (pt_Sensor != NULL)
   {
      opt_SensorInfo->pt_Sensor = pt_Sensor->pt_SensorDescriptor->pt_Sensor;
      if (pt_Sensor->pt_SensorNvmData != NULL)
      {
         opt_SensorInfo->s_Name = pt_Sensor->pt_SensorNvmData->ac_SensorName;
         opt_SensorInfo->u8_Flags = pt_Sensor->pt_SensorNvmData->u8_SensorFlags;
      }
      else
      {
         // called sensor_get_info() but m_sensor_detect() not finished
         opt_SensorInfo->s_Name = "none";
         opt_SensorInfo->u8_Flags = 0u;
      }
      opt_SensorInfo->u32_IicFreq = pt_Sensor->u32_IicFreq;
      opt_SensorInfo->u8_IicBus = pt_Sensor->u8_IicBus;
      opt_SensorInfo->u8_IicAddr = pt_Sensor->u8_IicAddr;
      t_Err = ESP_OK;
   }

   return t_Err;
}

esp_err_t sensor_set_setup(const uint16_t ou16_SensorIdx, const char * const oac_SensorName, const uint8_t ou8_SensorFlags)
{
   esp_err_t t_Err = sensor_data_set_name(ou16_SensorIdx, oac_SensorName);

   if (t_Err == ESP_OK)
   {
      t_Err = sensor_data_set_flags(ou16_SensorIdx, ou8_SensorFlags);
   }
   if (t_Err == ESP_OK)
   {
      t_Err = sensor_data_store();
   }

   return t_Err;
}

/**
 * call sensor process data (the sensor data processing task)
 */
esp_err_t sensor_process_data(const uint16_t ou16_SensorIdx)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   T_sensor_instance * const pt_Sensor = sensor_get_handle(ou16_SensorIdx);

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
   T_sensor_instance * const pt_Sensor = sensor_get_handle(ou16_SensorIdx);

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
   T_sensor_instance * const pt_Sensor = sensor_get_handle(ou16_SensorIdx);

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
