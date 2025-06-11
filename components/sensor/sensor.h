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

#define SENSOR_VALUE_INVALID           (-1.0E6f)      // invalid measurement value
#define SENSOR_TIMESTAMP_INVALID       (0xFFFFFFFFuL) // invalid measurement timestamp

/* -- Types --------------------------------------------------------------------------------------------------------- */

typedef struct
{
   float f32_Temperature;     // temperatur in degree centigrade
   float f32_Pressure;        // pressure in Pa
   float f32_Humidity;        // relative humidity in per cent
   float f32_CO2;             // CO2 concentration in PPM
   float f32_IAQ;             // air qualitiy index
   uint32_t u32_AgeSec;       // age of sensor data in seconds
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
   uint32_t u32_IicSclFreq;         // IIC SCL frequency
   uint8_t u8_IicBus;               // sensor IIC bus number
   uint8_t u8_IicAddr;              // sensor IIC bus address
} T_sensor_info;

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

uint16_t  sensor_get_count(void);
esp_err_t sensor_get_info(const uint16_t ou16_SensorIdx, const T_sensor_info ** opt_SensorInfo);
esp_err_t sensor_init(const uint16_t ou16_SensorIdx);
esp_err_t sensor_command(const uint16_t ou16_SensorIdx, const uint16_t ou16_Command);
esp_err_t sensor_process_data(const uint16_t ou16_SensorIdx);
esp_err_t sensor_get_data(const uint16_t ou16_SensorIdx, T_sensor_data * opt_SensorData);
void sensor_set_data_invalid(T_sensor_data * const opt_SensorData);

#endif /* SENSOR_H */
