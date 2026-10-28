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
#include "nvm_data.h"
#include <string.h>

/* -- Defines ------------------------------------------------------------------------------------------------------- */

#define SENSOR_NAME_MAX          16 // max. number of senor name entries in data base

/* -- Types --------------------------------------------------------------------------------------------------------- */

typedef struct
{
   uint16_t u16_SensorId;                          // sensor ID
   uint8_t  u8_IicBus;                             // IIC bus number
   char     ac_SensorName[SENSOR_NAME_LEN_MAX+1];  // sensor name string (max. 24 char + \0)
} T_sensor_name;

// sensor name data set
typedef struct
{
   uint32_t u32_CRC32;           // CRC32
   T_sensor_name t_SensorName;   // sensor name data
} T_sensor_name_data_crc;

_Static_assert(sizeof(T_sensor_name_data_crc) == NVM_DATA_SENSOR_SIZE, "T_sensor_name_data_crc must be of size NVM_DATA_SENSOR_SIZE byte");

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static uint32_t m_sensor_name_search_entry(const uint8_t ou8_IicBus, const uint16_t ou16_SensorId);
static char * m_sensor_name_create_default(const uint32_t ou32_Entry, const T_sensor_instance * opt_SensorInstance);


/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static const T_sensor_name mt_SensorNameEmpty =
{
   .u16_SensorId = 0u,
   .u8_IicBus = 0u,
   .ac_SensorName = { 0u },
};

static uint32_t mu32_NewData = 0u;
static T_sensor_name_data_crc mat_SensorNameDataCrc[SENSOR_NAME_MAX];

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * load sensor name data base from NVM
 */
esp_err_t sensor_name_load(void)
{
   for (uint32_t u32_Entry = 0u; u32_Entry < SENSOR_NAME_MAX; u32_Entry++)
   {
      const uint32_t u32_NvmAddr = NVM_DATA_SENSOR_ADDR + (u32_Entry * sizeof(T_sensor_name_data_crc));
      T_sensor_name_data_crc * const pt_SensorNameDataCrc = &mat_SensorNameDataCrc[u32_Entry];

      // read eeprom data
      const esp_err_t t_Err = nvm_data_load(u32_NvmAddr, sizeof(T_sensor_name_data_crc), pt_SensorNameDataCrc);
      if (t_Err != ESP_OK)
      {
         // invalidate data
         pt_SensorNameDataCrc->t_SensorName = mt_SensorNameEmpty;
      }
   }

   return ESP_OK;
}

/**
 * store sensor name data base into NVM
 */
esp_err_t sensor_name_store(void)
{
   for (uint32_t u32_Entry = 0u; u32_Entry < SENSOR_NAME_MAX; u32_Entry++)
   {
      const uint32_t u32_NewDataMsk = (1u << u32_Entry);

      if ((mu32_NewData & u32_NewDataMsk) != 0u)
      {
         const uint32_t u32_NvmAddr = NVM_DATA_SENSOR_ADDR + (u32_Entry * sizeof(T_sensor_name_data_crc));
         T_sensor_name_data_crc * const pt_SensorNameDataCrc = &mat_SensorNameDataCrc[u32_Entry];

         // store eeprom data
         const esp_err_t t_ErrStore = nvm_data_store(u32_NvmAddr, sizeof(T_sensor_name_data_crc), pt_SensorNameDataCrc);

         if (t_ErrStore == ESP_OK)
         {
            mu32_NewData &= ~u32_NewDataMsk;    // clear new data flag for this entry
         }
      }
   }

   return (mu32_NewData != 0u) ? ESP_FAIL : ESP_OK;
}

/**
 * return sensor name for sensor opt_SensorInstance
 */
char * sensor_name_get(const T_sensor_instance * opt_SensorInstance)
{
   const uint8_t u8_IicBus = opt_SensorInstance->u8_IicBus;
   const uint16_t u16_SensorId = opt_SensorInstance->pt_SensorDescriptor->pt_Sensor->u16_SensorId;
   char * ac_SensorName = NULL;

   // search for existing entry
   uint32_t u32_Entry = m_sensor_name_search_entry(u8_IicBus, u16_SensorId);

   if (u32_Entry < SENSOR_NAME_MAX)
   {
      ac_SensorName = mat_SensorNameDataCrc[u32_Entry].t_SensorName.ac_SensorName;
   }
   else
   {
      // if not exist: use next free entry and create default sensor name
      u32_Entry = m_sensor_name_search_entry(0u, 0u);

      if (u32_Entry < SENSOR_NAME_MAX)
      {
         ac_SensorName = m_sensor_name_create_default(u32_Entry, opt_SensorInstance);
      }
      else
      {
         // no empty entries: reuse 1st unused entry and create default sensor name
         // all entries in use: return NULL (should never happen, do not use >= SENSOR_NAME_MAX sensors per device)
         for (u32_Entry = 0u; u32_Entry < SENSOR_NAME_MAX; u32_Entry++)
         {
            T_sensor_name * const pt_SensorName = &mat_SensorNameDataCrc[u32_Entry].t_SensorName;
            T_sensor_instance * const pt_Sensor = sensor_data_get_handle_bus_id(pt_SensorName->u8_IicBus, pt_SensorName->u16_SensorId);

            if (pt_Sensor == NULL)
            {
               ac_SensorName = m_sensor_name_create_default(u32_Entry, opt_SensorInstance);
               break;
            }
         }
      }
   }

   return ac_SensorName;
}

/**
 * return sensor name for sensor addressed by ou16_SensorIdx
 */
char * sensor_name_get_idx(const uint16_t ou16_SensorIdx)
{
   T_sensor_instance * const pt_Sensor = sensor_data_get_handle_idx(ou16_SensorIdx);
   char * ac_SensorName = NULL;

   if (pt_Sensor != NULL)
   {
      ac_SensorName = sensor_name_get(pt_Sensor);
   }

   return ac_SensorName;
}

/**
 * set sensor name for sensor opt_SensorInstance
 */
esp_err_t sensor_name_set(const T_sensor_instance * opt_SensorInstance, const char * const oac_SensorName)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   const uint32_t u32_SensorNameLen = strnlen(oac_SensorName, SENSOR_NAME_LEN_MAX + 1u);

   if ((u32_SensorNameLen > 0u) && (u32_SensorNameLen <= SENSOR_NAME_LEN_MAX))
   {
      const uint8_t u8_IicBus = opt_SensorInstance->u8_IicBus;
      const uint16_t u16_SensorId = opt_SensorInstance->pt_SensorDescriptor->pt_Sensor->u16_SensorId;
      // search for existing entry
      const uint32_t u32_Entry = m_sensor_name_search_entry(u8_IicBus, u16_SensorId);

      if (u32_Entry < SENSOR_NAME_MAX)
      {
         T_sensor_name * const pt_SensorName = &mat_SensorNameDataCrc[u32_Entry].t_SensorName;
         (void)strncpy(pt_SensorName->ac_SensorName, oac_SensorName, SENSOR_NAME_LEN_MAX);

         mu32_NewData |= (1u << u32_Entry);  // set new data flag for this entry
      }
   }

   return t_Err;
}

/**
 * set sensor name for sensor addressed by ou16_SensorIdx
 */
esp_err_t sensor_name_set_idx(const uint16_t ou16_SensorIdx, const char * const oac_SensorName)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   T_sensor_instance * const pt_Sensor = sensor_data_get_handle_idx(ou16_SensorIdx);

   if (pt_Sensor != NULL)
   {
      t_Err = sensor_name_set(pt_Sensor, oac_SensorName);

   }

   return t_Err;
}

/**
 * search for existing name entry
 */
static uint32_t m_sensor_name_search_entry(const uint8_t ou8_IicBus, const uint16_t ou16_SensorId)
{
   uint32_t u32_Entry;

   for (u32_Entry = 0u; u32_Entry < SENSOR_NAME_MAX; u32_Entry++)
   {
      T_sensor_name * const pt_SensorName = &mat_SensorNameDataCrc[u32_Entry].t_SensorName;

      if ((pt_SensorName->u8_IicBus == ou8_IicBus) && (pt_SensorName->u16_SensorId == ou16_SensorId))
      {
         break;
      }
   }

   return u32_Entry;
}

/**
 * create default sensor name for name entry ou32_Entry
 */
static char * m_sensor_name_create_default(const uint32_t ou32_Entry, const T_sensor_instance * opt_SensorInstance)
{
   T_sensor_name * const pt_SensorName = &mat_SensorNameDataCrc[ou32_Entry].t_SensorName;
   const uint8_t u8_IicBus = opt_SensorInstance->u8_IicBus;
   uint32_t u32_SensorNameLen;

   // create default sensor name in format "<SensorType>_BUS<x>":
   // <SensorType>=sensor type string, <x>=I2C bus number
   const char * const s_SensorType = opt_SensorInstance->pt_SensorDescriptor->pt_Sensor->s_Type;
   (void)sprintf(pt_SensorName->ac_SensorName, "%s_BUS%u", s_SensorType, u8_IicBus);
   u32_SensorNameLen = strnlen(pt_SensorName->ac_SensorName, SENSOR_NAME_LEN_MAX +1u);
   // fill unused string array data with \0
   (void)memset(&pt_SensorName->ac_SensorName[u32_SensorNameLen+1u], 0, SENSOR_NAME_LEN_MAX-u32_SensorNameLen);

   // setup sensor name data
   pt_SensorName->u16_SensorId = opt_SensorInstance->pt_SensorDescriptor->pt_Sensor->u16_SensorId;
   pt_SensorName->u8_IicBus = u8_IicBus;

   mu32_NewData |= (1u << ou32_Entry);  // set new data flag for this entry

   return pt_SensorName->ac_SensorName;
}
