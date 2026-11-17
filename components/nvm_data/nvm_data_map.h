//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  nvm_data_map.h
   \brief defines the NVM memory map
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef NVM_DATA_MAP_H
#define NVM_DATA_MAP_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "iic_eeprom_size.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

#define NVM_DATA_WIFI_ADDR          0u          // start address of WiFi data set
#define NVM_DATA_WIFI_SIZE          160u        // size of WiFi data set
#define NVM_DATA_OUT_ADDR           256u        // start address of output data sets
#define NVM_DATA_OUT_SIZE           32u         // size of output data set (per output channel, 8 channels available)
#define NVM_DATA_SENSOR_ADDR        512u        // start address of sensor data sets
#define NVM_DATA_SENSOR_SIZE        32u         // size of sensor data set (per sensor, 16 name entries available)
#define NVM_DATA_MQTT_ADDR          1024u       // start address of MQTT data set
#define NVM_DATA_MQTT_SIZE          256u        // size of MQTT data set
#define NVM_DATA_SYSTEM_ADDR        1536u       // start address of system data set
#define NVM_DATA_SYSTEM_SIZE        32u         // size of system data set

#define NVM_DATA_MIRROR_ADDR        4096u       // start address of NVM data mirror


_Static_assert((NVM_DATA_WIFI_SIZE % IIC_EEPROM_PAGE_SIZE) == 0, "NVM_DATA_WIFI_SIZE must be multible of IIC_EEPROM_PAGE_SIZE");
_Static_assert((NVM_DATA_OUT_SIZE % IIC_EEPROM_PAGE_SIZE) == 0, "NVM_DATA_OUT_SIZE must be multible of IIC_EEPROM_PAGE_SIZE");
_Static_assert((NVM_DATA_SENSOR_SIZE % IIC_EEPROM_PAGE_SIZE) == 0, "NVM_DATA_SENSOR_SIZE must be multible of IIC_EEPROM_PAGE_SIZE");
_Static_assert((NVM_DATA_MQTT_SIZE % IIC_EEPROM_PAGE_SIZE) == 0, "NVM_DATA_MQTT_SIZE must be multible of IIC_EEPROM_PAGE_SIZE");
_Static_assert((NVM_DATA_SYSTEM_SIZE % IIC_EEPROM_PAGE_SIZE) == 0, "NVM_DATA_SYSTEM_SIZE must be multible of IIC_EEPROM_PAGE_SIZE");

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- Implementation ------------------------------------------------------------------------------------------------ */

#endif /* NVM_DATA_MAP_H */
