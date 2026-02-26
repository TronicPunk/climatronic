//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor_name.c
   \brief provide a valid sensor name. If there is no valid sensor name in the sensor data then
          create a default name based on sensor type and IIC bus + IIC address information
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */

#include "sensor_name.h"
#include "sensor_data.h"
#include <string.h>

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * return sensor name for address ou8_IicAddr at IIC bus ou8_IicBus
 */
char * sensor_name_create(const T_sensor_instance * opt_SensorInstance)
{
   const uint8_t u8_IicBus = opt_SensorInstance->u8_IicBus;
   const uint8_t u8_IicAddr = opt_SensorInstance->u8_IicAddr;
   char * pt_SensorName = (char*)sensor_data_get_name(u8_IicBus, u8_IicAddr);

   if (pt_SensorName == NULL)
   {
      // create default sensor name in format "<SensorType>_I2C<x>_0x<XX>":
      // <SensorType>=sensor type string, <x>=I2C bus number, <XX>=I2C device address
      const char * const s_SensorType = opt_SensorInstance->pt_SensorDescriptor->pt_Sensor->s_Type;
      char * const pt_SensorNameDef = malloc(strlen(s_SensorType) + 12u);

      if (pt_SensorNameDef != NULL)
      {
         (void)sprintf(pt_SensorNameDef, "%s_I2C%u_0x%02X", s_SensorType, opt_SensorInstance->u8_IicBus,
                       opt_SensorInstance->u8_IicAddr);
         pt_SensorName = pt_SensorNameDef;
      }
   }

   return pt_SensorName;
}

/**
 * free memory for sensor name
 */
void sensor_name_free(T_sensor_instance * const opt_SensorInstance)
{
   if (opt_SensorInstance->s_SensorName != NULL)
   {
      // check if this is allocated memory
      const char * const s_SensorName = sensor_data_get_name(opt_SensorInstance->u8_IicBus, opt_SensorInstance->u8_IicAddr);
      if (opt_SensorInstance->s_SensorName != s_SensorName)
      {
         free(opt_SensorInstance->s_SensorName);
         opt_SensorInstance->s_SensorName = NULL;
      }
   }
}
