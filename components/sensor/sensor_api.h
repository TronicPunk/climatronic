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
#define SENSOR_IIC_ADDR_MAX      3u    // max. number of supported IIC addresses per sensor
#define SENSOR_NAME_LEN_MAX      24    // max. length of sensor name string (without \0)
#define SENSOR_FLAGS_REMOTE_RD   0x01u // Sensor flag: Sensor data are remote readable (visable)

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

typedef struct
{
   uint16_t u16_SensorId;                             // internal sensor ID + IIC bus number
   uint8_t  u8_SensorFlags;                           // sensor flags
   char     ac_SensorName[SENSOR_NAME_LEN_MAX+1];     // sensor name string (max. 24 char + \0)
} T_sensor_nvm_data;

typedef struct t_sensor_instance
{
   uint64_t u64_CallTimeUs;                           // next call (absolute system time in usec)
   T_sensor_data t_SensorData;                        // sensor data sample
   const T_sensor_descriptor * pt_SensorDescriptor;   // pointer to sensor descriptor
   T_sensor_nvm_data * pt_SensorNvmData;              // pointer to sensor NVM data
   void * pv_Handle;                                  // sensor handle with driver specific information
   uint32_t u32_Parameter;                            // parameter/result for sensor commands
   uint32_t u32_IicFreq;                              // connected IIC SCL frequency
   uint16_t u16_SensorState;                          // sensor state (command processing)
   uint8_t u8_IicBus;                                 // connected IIC bus
   uint8_t u8_IicAddr;                                // sensor IIC bus address
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

T_sensor_instance* sensor_get_handle(const uint16_t ou16_SensorIdx);

#endif /* SENSOR_API_H */
