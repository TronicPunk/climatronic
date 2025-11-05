//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor.c
   \brief provide functions for sensor communication
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "sensor_data.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define SENSOR_COUNT    (6u)  // number of connected sensors

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

// array of sensor instance data
static T_sensor_instance mat_SensorInstance[SENSOR_COUNT];

// array of const sensor descriptors
static const T_sensor_def mat_SensorList[SENSOR_COUNT] =
{
   {
      .t_SensorInfo =
      {
         .pt_Sensor = &gt_BME680_type,                // Sensor type BME680
         .s_Name = "Aufzucht1_Sensor1",               // Sensor name or location
         .u32_IicSclFreq = 400000u,                   // IIC SCL frequency
         .u8_IicBus = 0u,                             // IIC bus number
         .u8_IicAddr = BME68X_I2C_ADDR_HIGH           // IIC bus address
      },
      .pt_SensorApi = &gt_BME68x_api,                 // pointer to sensor API
      .u32_UpdateRateUs = 3000000u,                   // update rate 3sec
      .pt_SensorInstance = &mat_SensorInstance[0],    // pointer to sensor instance data storage
   },
   {
      .t_SensorInfo =
      {
         .pt_Sensor = &gt_SCD41_type,                 // Sensor type SCD41
         .s_Name = "Aufzucht1_CO2_Sensor1",           // Sensor name or location
         .u32_IicSclFreq = 400000u,                   // IIC SCL frequency
         .u8_IicBus = 0u,                             // IIC bus number
         .u8_IicAddr = SCD41_I2C_ADDR_62              // IIC bus address
      },
      .pt_SensorApi = &gt_SCD4x_api,                  // pointer to sensor API
      .u32_UpdateRateUs = 5000000u,                   // update rate 5sec
      .pt_SensorInstance = &mat_SensorInstance[1],    // pointer to sensor instance data storage
   },
   {
      .t_SensorInfo =
      {
         .pt_Sensor = &gt_BME680_type,                // Sensor type BME680
         .s_Name = "Aufzucht1_Sensor2",               // Sensor name or location
         .u32_IicSclFreq = 400000u,                   // IIC SCL frequency
         .u8_IicBus = 1u,                             // IIC bus number
         .u8_IicAddr = BME68X_I2C_ADDR_HIGH           // IIC bus address
      },
      .pt_SensorApi = &gt_BME68x_api,                 // pointer to sensor API
      .u32_UpdateRateUs = 3000000u,                   // update rate 3sec
      .pt_SensorInstance = &mat_SensorInstance[2],    // pointer to sensor instance data storage
   },
   {
      .t_SensorInfo =
      {
         .pt_Sensor = &gt_SCD41_type,                 // Sensor type SCD41
         .s_Name = "Aufzucht1_CO2_Sensor2",           // Sensor name or location
         .u32_IicSclFreq = 400000u,                   // IIC SCL frequency
         .u8_IicBus = 1u,                             // IIC bus number
         .u8_IicAddr = SCD41_I2C_ADDR_62              // IIC bus address
      },
      .pt_SensorApi = &gt_SCD4x_api,                  // pointer to sensor API
      .u32_UpdateRateUs = 5000000u,                   // update rate 5sec
      .pt_SensorInstance = &mat_SensorInstance[3],    // pointer to sensor instance data storage
   },
   {
      .t_SensorInfo =
      {
         .pt_Sensor = &gt_BME280_type,                // Sensor type BME280
         .s_Name = "Zuluft_Sensor1",                  // Sensor name or location
         .u32_IicSclFreq = 400000u,                   // IIC SCL frequency
         .u8_IicBus = 2u,                             // IIC bus number
         .u8_IicAddr = BME280_I2C_ADDR_PRIM           // IIC bus address
      },
      .pt_SensorApi = &gt_BME280_api,                 // pointer to sensor API
      .u32_UpdateRateUs = 1000000u,                   // update rate 1sec
      .pt_SensorInstance = &mat_SensorInstance[4],    // pointer to sensor instance data storage
   },
   {
      .t_SensorInfo =
      {
         .pt_Sensor = &gt_BME280_type,                // Sensor type BME280
         .s_Name = "Aussen_Sensor1",                  // Sensor name or location
         .u32_IicSclFreq = 400000u,                   // IIC SCL frequency
         .u8_IicBus = 3u,                             // IIC bus number
         .u8_IicAddr = BME280_I2C_ADDR_PRIM           // IIC bus address
      },
      .pt_SensorApi = &gt_BME280_api,                 // pointer to sensor API
      .u32_UpdateRateUs = 1000000u,                   // update rate 1sec
      .pt_SensorInstance = &mat_SensorInstance[5],    // pointer to sensor instance data storage
   }
};


/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * return number of registered sensors
 */
uint16_t sensor_data_get_count(void)
{
   return SENSOR_COUNT;
}

/**
 * Return sensor handle. Return NULL if ou16_SensorIdx is out of range
 */
const T_sensor_def* sensor_data_get_handle(const uint16_t ou16_SensorIdx)
{
   const T_sensor_def* pt_SensorHandle;

   if (ou16_SensorIdx < SENSOR_COUNT)
   {
      pt_SensorHandle = &mat_SensorList[ou16_SensorIdx];
   }
   else
   {
      pt_SensorHandle = NULL;
   }

   return pt_SensorHandle;
}
