//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor.h
   \brief provide functions for sensor communication
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef SENSOR_H
#define SENSOR_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "esp_types.h"
#include "esp_err.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define SENSOR_CAP_TEMPERATURE         (1u << 0u)     // sensor capability: temperature
#define SENSOR_CAP_PRESSURE            (1u << 1u)     // sensor capability: pressure
#define SENSOR_CAP_HUMIDITY            (1u << 2u)     // sensor capability: humidity
#define SENSOR_CAP_CO2                 (1u << 3u)     // sensor capability: carbon dioxyde
#define SENSOR_CAP_IAQ                 (1u << 4u)     // sensor capability: index for air quality
#define SENSOR_CAP_MODE_SLEEP          (1u << 12u)    // sensor capability: measurement mode sleep (mandatory operation mode)
#define SENSOR_CAP_MODE_SINGLE         (1u << 13u)    // sensor capability: measurement mode single (mandatory operation mode)
#define SENSOR_CAP_MODE_CYCLIC         (1u << 14u)    // sensor capability: measurement mode cyclic (optional operation mode)

#define SENSOR_MODE_SLEEP              (0x00u)        // mandatory operation mode
#define SENSOR_MODE_SINGLE_MEASURE     (0x01u)        // mandatory operation mode
#define SENSOR_MODE_CYCLIC_MEASURE     (0x02u)        // optional  operation mode
#define SENSOR_MODE_COUNT              (3u)           // max. number of sensor operation modes

#define SENSOR_VALUE_INVALID           (-500.0f)      // invalid measurement value
#define SENSOR_TIMESTAMP_INVALID       (0xFFFFFFFFuL) // invalid measurement timestamp

#define SENSOR_COMMAND_GET_STATE       (0x0000u)      // read command state
#define SENSOR_COMMAND_RESET           (0x0001u)      // reset sensor (TBD)
#define SENSOR_COMMAND_CALIBRATE_CO2   (0x0002u)      // calibrate CO2 sensor

/* -- Types --------------------------------------------------------------------------------------------------------- */

typedef struct
{
   float f32_Temperature;     // temperatur in degree centigrade
   float f32_Pressure;        // pressure in hPa
   float f32_Humidity;        // relative humidity in per cent
   float f32_CO2;             // CO2 concentration in PPM
   float f32_IAQ;             // air qualitiy index
   uint32_t u32_TimeStampSec; // sensor data timestamp (system time in seconds)
} T_sensor_data;

typedef struct
{
   const char * s_Type;       // sensor type
   uint16_t u16_SensorCaps;   // sensor capabilities
} T_sensor_type;

typedef struct
{
   const T_sensor_type* pt_Sensor;  // sensor type info
   const char * s_Name;             // sensor name
   uint32_t u32_IicFreq;            // IIC clock frequency
   uint8_t u8_IicBus;               // sensor IIC bus number
   uint8_t u8_IicAddr;              // sensor IIC bus address
} T_sensor_info;

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

esp_err_t sensor_init_task(void);   // init the sensor measurement process
esp_err_t sensor_detect(void);      // scan all I2C busses and init found sensors
uint16_t  sensor_get_count(void);   // get number of initialized sensors
esp_err_t sensor_get_info(const uint16_t ou16_SensorIdx, T_sensor_info * const opt_SensorInfo);
esp_err_t sensor_process_data(const uint16_t ou16_SensorIdx);
esp_err_t sensor_get_data(const uint16_t ou16_SensorIdx, T_sensor_data * const opt_SensorData);
esp_err_t sensor_command(const uint16_t ou16_SensorIdx, const uint16_t ou16_Command, uint32_t * const  opu32_Parameter);
void      sensor_set_data_invalid(T_sensor_data * const opt_SensorData);

#endif /* SENSOR_H */
