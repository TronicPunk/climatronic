//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor.c
   \brief provide functions for sensor communication
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "sensor_data.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define I2C_BUS_INVALID    (0xFFu)  // number of availabel I2C sensor busses
#define SENSOR_NAME_COUNT  (6u)     // number of named I2C sensors

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static uint16_t mu16_SensorCount = 0u;  // number of installed (detected) sensors
static T_sensor_instance * mpt_SensorList = NULL;
static T_sensor_instance * mpt_SensorListEnd = NULL;


const T_sensor_bus_init gat_SensorbusInit[ICC_BUS_COUNT +1u] =
{
   { .u8_IicBus = 0u, .u32_IicFreqHz = 400000u },
   { .u8_IicBus = 1u, .u32_IicFreqHz = 400000u },
   { .u8_IicBus = 2u, .u32_IicFreqHz = 400000u },
   { .u8_IicBus = 3u, .u32_IicFreqHz = 400000u },
   { .u8_IicBus = 4u, .u32_IicFreqHz = 400000u },
   { .u8_IicBus = 5u, .u32_IicFreqHz = 400000u },
   // sensor bus init list terminator
   { .u8_IicBus = I2C_BUS_INVALID, .u32_IicFreqHz = 0u }
};

const T_sensor_descriptor * gapt_SensorTypes[SENSOR_TYPE_COUNT +1u] =
{
   &gt_BME280_descriptor,
   &gt_BME680_descriptor,
   &gt_SCD41_descriptor,
   // sensor descriptor list terminator
   NULL
};

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * add a new sensor instance to the sensor list
 */
void sensor_data_register_sensor(T_sensor_instance * const opt_NewSensor)
{
   opt_NewSensor->pt_Next = NULL;

   if (mpt_SensorList == NULL)
   {
      mpt_SensorList = opt_NewSensor;
   }
   else
   {
      mpt_SensorListEnd->pt_Next = opt_NewSensor;
   }

   mpt_SensorListEnd = opt_NewSensor;
   mu16_SensorCount++;
}


/**
 * return number of registered sensors
 */
uint16_t sensor_data_get_count(void)
{
   return mu16_SensorCount;
}

/**
 * Return sensor handle. Return NULL if ou16_SensorIdx is out of range
 */
T_sensor_instance* sensor_data_get_handle_idx(const uint16_t ou16_SensorIdx)
{
   T_sensor_instance* pt_SensorInstance = NULL;

   if ((ou16_SensorIdx < mu16_SensorCount) && (mu16_SensorCount > 0u))
   {
      pt_SensorInstance = mpt_SensorList;

      for (uint16_t u16_SensorCnt = 0u; u16_SensorCnt < ou16_SensorIdx; u16_SensorCnt++)
      {
         pt_SensorInstance = pt_SensorInstance->pt_Next;
      }
   }

   return pt_SensorInstance;
}

/**
 * Return sensor handle from IIC bus number and senor ID. Return NULL if sensor does not exist
 */
T_sensor_instance* sensor_data_get_handle_bus_id(const uint8_t ou8_IicBus, const uint16_t ou16_SensorId)
{
   T_sensor_instance* pt_SensorInstance = mpt_SensorList;

   while (pt_SensorInstance != NULL)
   {
      const uint8_t u8_IicBus = pt_SensorInstance->u8_IicBus;
      const uint16_t u16_SensorId = pt_SensorInstance->pt_SensorDescriptor->pt_Sensor->u16_SensorId;

      if ((u8_IicBus == ou8_IicBus) && (u16_SensorId == ou16_SensorId))
      {
         break;
      }
      else
      {
         pt_SensorInstance = pt_SensorInstance->pt_Next;
      }
   }

   return pt_SensorInstance;
}