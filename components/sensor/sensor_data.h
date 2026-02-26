//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor_data.h
   \brief provide data for all available sensors
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "sensor_api.h"
#include "bme280_interface.h"
#include "bme68x_interface.h"
#include "scd4x_interface.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define ICC_BUS_COUNT      4u    // number of available IIC busses
#define SENSOR_TYPE_COUNT  3u    // number of different supported IIC sensors

/* -- Types --------------------------------------------------------------------------------------------------------- */

typedef struct
{
   uint8_t u8_IicBus;               // IIC bus number that should be probed
   uint32_t u32_IicFreqHz;          // IIC bus clock frequency in Hz
} T_sensor_bus_init;


/* -- Global Variables ---------------------------------------------------------------------------------------------- */

extern const T_sensor_bus_init gat_SensorbusInit[ICC_BUS_COUNT +1u];
extern const T_sensor_descriptor * gapt_SensorTypes[SENSOR_TYPE_COUNT +1u];


/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

const char* sensor_data_get_name(const uint8_t ou8_IicBus, const uint8_t ou8_IicAddr);
void sensor_data_register_sensor(T_sensor_instance * const opt_NewSensor);
uint16_t sensor_data_get_count(void);
T_sensor_instance* sensor_data_get_handle(const uint16_t ou16_SensorIdx);

#endif /* SENSOR_DATA_H */
