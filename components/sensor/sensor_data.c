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

typedef struct
{
   const char * s_Name;             // sensor name
   uint8_t u8_IicBus;               // sensor IIC bus number
   uint8_t u8_IicAddr;              // sensor IIC bus address
} T_sensor_name;


/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static uint16_t mu16_SensorCount = 0u;  // number of installed (detected) sensors
T_sensor_instance * gpt_SensorList = NULL;
T_sensor_instance * mpt_SensorListEnd = NULL;


const T_sensor_bus_init gat_SensorbusInit[ICC_BUS_COUNT +1u] =
{
   { .u8_IicBus = 0u, .u32_IicFreqHz = 400000u },
   { .u8_IicBus = 1u, .u32_IicFreqHz = 400000u },
   { .u8_IicBus = 2u, .u32_IicFreqHz = 400000u },
   { .u8_IicBus = 3u, .u32_IicFreqHz = 400000u },
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

// array of named sensors
static const T_sensor_name mat_SensorNameList[SENSOR_NAME_COUNT +1u] =
{
   {
      .u8_IicBus = 0u,                       // IIC bus number
      .u8_IicAddr = BME68X_I2C_ADDR_HIGH,    // IIC bus address
      .s_Name = "Aufzucht1_Sensor1"          // Sensor name or location
   },
   {
      .u8_IicBus = 0u,                       // IIC bus number
      .u8_IicAddr = SCD41_I2C_ADDR_62,       // IIC bus address
      .s_Name = "Aufzucht1_CO2_Sensor1"      // Sensor name or location
   },
   {
      .u8_IicBus = 1u,                       // IIC bus number
      .u8_IicAddr = BME68X_I2C_ADDR_HIGH,    // IIC bus address
      .s_Name = "Aufzucht1_Sensor2"          // Sensor name or location
   },
   {
      .u8_IicBus = 1u,                       // IIC bus number
      .u8_IicAddr = SCD41_I2C_ADDR_62,       // IIC bus address
      .s_Name = "Aufzucht1_CO2_Sensor2"      // Sensor name or location
   },
   {
      .u8_IicBus = 2u,                       // IIC bus number
      .u8_IicAddr = BME280_I2C_ADDR_PRIM,    // IIC bus address
      .s_Name = "Zuluft_Sensor1"             // Sensor name or location
   },
   {
      .u8_IicBus = 3u,                       // IIC bus number
      .u8_IicAddr = BME280_I2C_ADDR_PRIM,    // IIC bus address
      .s_Name = "Aussen_Sensor1"             // Sensor name or location
   },
   // sensor name list terminator
   {
      .u8_IicBus = I2C_BUS_INVALID,          // IIC bus number invalid
      .u8_IicAddr = 0u,                      //
      .s_Name = NULL                         //
   }
};


/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * return name for sensor with address ou8_IicAddr at IIC bus ou8_IicBus
 */
const char* sensor_data_get_name(const uint8_t ou8_IicBus, const uint8_t ou8_IicAddr)
{
   const T_sensor_name * pt_SensorName = &mat_SensorNameList[0];

   while ((pt_SensorName->u8_IicBus != I2C_BUS_INVALID) && (pt_SensorName->s_Name != NULL))
   {
      if ((pt_SensorName->u8_IicBus == ou8_IicBus) && (pt_SensorName->u8_IicAddr == ou8_IicAddr))
      {
         break;
      }
      else
      {
         pt_SensorName = &pt_SensorName[1];
      }
   }

   return pt_SensorName->s_Name;
}


/**
 * add a new sensor instance to the sensor list
 */
void sensor_data_register_sensor(T_sensor_instance * const opt_NewSensor)
{
   if (gpt_SensorList == NULL)
   {
      gpt_SensorList = opt_NewSensor;
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
T_sensor_instance* sensor_data_get_handle(const uint16_t ou16_SensorIdx)
{
   T_sensor_instance* pt_SensorInstance = NULL;

   if ((ou16_SensorIdx < mu16_SensorCount) && (mu16_SensorCount > 0u))
   {
      pt_SensorInstance = gpt_SensorList;

      for (uint16_t u16_SensorCnt = 0u; u16_SensorCnt < ou16_SensorIdx; u16_SensorCnt++)
      {
         pt_SensorInstance = pt_SensorInstance->pt_Next;
      }
   }

   return pt_SensorInstance;
}
