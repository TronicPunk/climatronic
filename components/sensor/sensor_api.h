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
#define SENSOR_IIC_ADDR_MAX   7u    // max. number of supported IIC addresses per sensor

/* -- Types --------------------------------------------------------------------------------------------------------- */

typedef struct
{
   const T_sensor_type* pt_Sensor;                    // sensor type info
   const struct _sensor_api * const pt_SensorApi;     // generic sensor API functions
   const uint32_t u32_UpdateRateMinUs;                // max. sensor update rate in us
   const uint32_t u32_IicFreqMaxHz;                   // max. sensor IIC clock frequency
   const uint8_t u8_IicAddrCount;                     // number of supported IIC addresses
   const uint8_t au8_IicAddr[SENSOR_IIC_ADDR_MAX];    // array of supported IIC addresses
} T_sensor_descriptor;

typedef struct t_sensor_instance
{
   struct t_sensor_instance * pt_Next;                // pointer to next sensor instance, NULL for last sensor instance
   uint64_t u64_CallTimeUs;                           // next call (absolute system time in usec)
   T_sensor_data t_SensorData;                        // sensor data sample
   char * s_SensorName;                               // sensor name
   void * pv_Handle;                                  // sensor handle with driver specific information
   const T_sensor_descriptor * pt_SensorDescriptor;   // pointer to sensor descriptor
   uint32_t u32_Parameter;                            // parameter/result for sensor commands
   uint16_t u16_SensorState;                          // sensor state (command processing)
   uint8_t u8_IicBus;                                 // connected IIC bus
   uint8_t u8_IicAddr;                                // sensor IIC bus address
   uint32_t u32_IicFreq;                              // connected IIC SCL frequency
} T_sensor_instance;

typedef struct _sensor_api
{
   esp_err_t (*pr_SensorInit)(const uint8_t ou8_IicBus, const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                              T_sensor_instance ** oppt_Sensor);
   esp_err_t (*pr_SensorProcessData)(T_sensor_instance * const opt_Sensor);
   esp_err_t (*pr_SensorGetData)(T_sensor_instance * const opt_Sensor, T_sensor_data * const opt_SensorData);
   esp_err_t (*pr_SensorCommand)(T_sensor_instance * const opt_Sensor, const uint16_t ou16_Command, uint32_t * const  opu32_Parameter);
} T_sensor_api;

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

#endif /* SENSOR_API_H */
