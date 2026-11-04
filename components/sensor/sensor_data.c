//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor.c
   \brief provide functions for sensor communication
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "sensor_data.h"
#include "nvm_data.h"
#include <string.h>

/* -- Defines ------------------------------------------------------------------------------------------------------- */

#define SENSOR_NVM_DATA_MAX    16    // max. number of sensor entries in data base

/* -- Types --------------------------------------------------------------------------------------------------------- */

// sensor nvm data set incl. CRC
typedef struct
{
   uint32_t u32_CRC32;                 // CRC32
   T_sensor_nvm_data t_SensorNvmData;  // sensor nvm data
} T_sensor_nvm_data_crc;

_Static_assert(sizeof(T_sensor_nvm_data_crc) == NVM_DATA_SENSOR_SIZE, "T_sensor_nvm_data_crc must be of size NVM_DATA_SENSOR_SIZE byte");

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static T_sensor_instance* m_sensor_data_get_handle_by_sensor_id(const uint16_t ou16_SensorId);
static uint16_t m_sensor_data_get_sensor_id(const T_sensor_instance * opt_Sensor);
static uint32_t m_sensor_data_search_entry(const uint16_t ou16_SensorId);
static T_sensor_nvm_data * m_sensor_data_create_default(const uint32_t ou32_Entry, const T_sensor_instance * opt_Sensor);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static const T_sensor_nvm_data mt_SensorNvmDataEmpty =
{
   .u16_SensorId = 0u,
   .u8_SensorFlags = 0u,
   .ac_SensorName = { 0u },
};

static uint32_t mu32_NewData = 0u;
static T_sensor_nvm_data_crc mat_SensorNvmDataCrc[SENSOR_NVM_DATA_MAX];


/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * load sensor data base from NVM
 */
esp_err_t sensor_data_load(void)
{
   for (uint32_t u32_Entry = 0u; u32_Entry < SENSOR_NVM_DATA_MAX; u32_Entry++)
   {
      const uint32_t u32_NvmAddr = NVM_DATA_SENSOR_ADDR + (u32_Entry * sizeof(T_sensor_nvm_data_crc));
      T_sensor_nvm_data_crc * const pt_SensorNvmDataCrc = &mat_SensorNvmDataCrc[u32_Entry];

      // read eeprom data
      const esp_err_t t_Err = nvm_data_load(u32_NvmAddr, sizeof(T_sensor_nvm_data_crc), pt_SensorNvmDataCrc);
      if (t_Err != ESP_OK)
      {
         // invalidate data
         pt_SensorNvmDataCrc->t_SensorNvmData = mt_SensorNvmDataEmpty;
         mu32_NewData |= (1u << u32_Entry);  // set new data flag for this entry
      }
   }

   return ESP_OK;
}

/**
 * store sensor data base into NVM
 */
esp_err_t sensor_data_store(void)
{
   for (uint32_t u32_Entry = 0u; u32_Entry < SENSOR_NVM_DATA_MAX; u32_Entry++)
   {
      const uint32_t u32_NewDataMsk = (1u << u32_Entry);

      if ((mu32_NewData & u32_NewDataMsk) != 0u)
      {
         const uint32_t u32_NvmAddr = NVM_DATA_SENSOR_ADDR + (u32_Entry * sizeof(T_sensor_nvm_data_crc));
         T_sensor_nvm_data_crc * const pt_SensorNvmDataCrc = &mat_SensorNvmDataCrc[u32_Entry];

         // store eeprom data
         const esp_err_t t_ErrStore = nvm_data_store(u32_NvmAddr, sizeof(T_sensor_nvm_data_crc), pt_SensorNvmDataCrc);

         if (t_ErrStore == ESP_OK)
         {
            mu32_NewData &= ~u32_NewDataMsk;    // clear new data flag for this entry
         }
      }
   }

   return (mu32_NewData != 0u) ? ESP_FAIL : ESP_OK;
}

/**
 * return sensor data for sensor adressed by ou16_SensorIdx
 */
T_sensor_nvm_data * sensor_data_get(const uint16_t ou16_SensorIdx)
{
   T_sensor_nvm_data * pt_SensorNvmData = NULL;
   T_sensor_instance * const pt_Sensor = sensor_get_handle(ou16_SensorIdx);

   if (pt_Sensor != NULL)
   {
      const uint16_t u16_SensorId = m_sensor_data_get_sensor_id(pt_Sensor);

      // search for existing entry
      uint32_t u32_Entry = m_sensor_data_search_entry(u16_SensorId);

      if (u32_Entry < SENSOR_NVM_DATA_MAX)
      {
         pt_SensorNvmData = &mat_SensorNvmDataCrc[u32_Entry].t_SensorNvmData;
      }
      else
      {
         // if not exist: use next free entry and create default sensor name
         u32_Entry = m_sensor_data_search_entry(0u);

         if (u32_Entry < SENSOR_NVM_DATA_MAX)
         {
            pt_SensorNvmData = m_sensor_data_create_default(u32_Entry, pt_Sensor);
         }
         else
         {
            // no empty entries: reuse 1st unused entry and create default sensor data
            // all entries in use: return NULL (should never happen, do not use >= SENSOR_NVM_DATA_MAX sensors per device)
            for (u32_Entry = 0u; u32_Entry < SENSOR_NVM_DATA_MAX; u32_Entry++)
            {
               const uint16_t u16_SensorId = mat_SensorNvmDataCrc[u32_Entry].t_SensorNvmData.u16_SensorId;
               T_sensor_instance * const pt_Sensor = m_sensor_data_get_handle_by_sensor_id(u16_SensorId);

               if (pt_Sensor == NULL)
               {
                  pt_SensorNvmData = m_sensor_data_create_default(u32_Entry, pt_Sensor);
                  break;
               }
            }
         }
      }
   }

   return pt_SensorNvmData;
}

/**
 * Return sensor handle from internal sensor ID. Return NULL if sensor does not exist
 */
static T_sensor_instance* m_sensor_data_get_handle_by_sensor_id(const uint16_t ou16_SensorId)
{
   T_sensor_instance * pt_SensorFound = NULL;
   const uint16_t u16_SensorCount = sensor_get_count();

   for (uint32_t u32_Sensor = 0u; u32_Sensor < u16_SensorCount; u32_Sensor++)
   {
      T_sensor_instance * const pt_Sensor = sensor_get_handle(u32_Sensor);
      const uint16_t u16_SensorId = m_sensor_data_get_sensor_id(pt_Sensor);
      if (u16_SensorId == ou16_SensorId)
      {
         pt_SensorFound = pt_Sensor;
         break;
      }
   }

   return pt_SensorFound;
}

/**
 * set sensor name for sensor addressed by ou16_SensorIdx
 */
esp_err_t sensor_data_set_name(const uint16_t ou16_SensorIdx, const char * const oac_SensorName)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   T_sensor_instance * const pt_Sensor = sensor_get_handle(ou16_SensorIdx);
   const uint32_t u32_SensorNameLen = strnlen(oac_SensorName, SENSOR_NAME_LEN_MAX + 1u);

   if ((pt_Sensor != NULL) && (u32_SensorNameLen > 0u) && (u32_SensorNameLen <= SENSOR_NAME_LEN_MAX))
   {
      const uint16_t u16_SensorId = m_sensor_data_get_sensor_id(pt_Sensor);
      const uint32_t u32_Entry = m_sensor_data_search_entry(u16_SensorId);

      if (u32_Entry < SENSOR_NVM_DATA_MAX)
      {
         T_sensor_nvm_data * pt_SensorNvmData = &mat_SensorNvmDataCrc[u32_Entry].t_SensorNvmData;
         if (strncmp(pt_SensorNvmData->ac_SensorName, oac_SensorName, SENSOR_NAME_LEN_MAX) != 0)
         {
            (void)strncpy(pt_SensorNvmData->ac_SensorName, oac_SensorName, SENSOR_NAME_LEN_MAX);
            pt_SensorNvmData->ac_SensorName[SENSOR_NAME_LEN_MAX] = '\0';
            mu32_NewData |= (1u << u32_Entry);  // set new data flag for this entry
         }
         t_Err = ESP_OK;
      }
   }

   return t_Err;
}

/**
 * set sensor flags for sensor addressed by ou16_SensorIdx
 */
esp_err_t sensor_data_set_flags(const uint16_t ou16_SensorIdx, const uint8_t ou8_SensorFlags)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   T_sensor_instance * const pt_Sensor = sensor_get_handle(ou16_SensorIdx);

   if (pt_Sensor != NULL)
   {
      const uint16_t u16_SensorId = m_sensor_data_get_sensor_id(pt_Sensor);
      const uint32_t u32_Entry = m_sensor_data_search_entry(u16_SensorId);

      if (u32_Entry < SENSOR_NVM_DATA_MAX)
      {
         T_sensor_nvm_data * pt_SensorNvmData = &mat_SensorNvmDataCrc[u32_Entry].t_SensorNvmData;
         if (pt_SensorNvmData->u8_SensorFlags != ou8_SensorFlags)
         {
            pt_SensorNvmData->u8_SensorFlags = ou8_SensorFlags;
            mu32_NewData |= (1u << u32_Entry);  // set new data flag for this entry
         }
         t_Err = ESP_OK;
      }
   }

   return t_Err;
}

/**
 * get internal sensor ID as data set identifier
 */
static uint16_t m_sensor_data_get_sensor_id(const T_sensor_instance * opt_Sensor)
{
   const uint16_t u16_SensorId = opt_Sensor->pt_SensorDescriptor->pt_Sensor->u16_SensorId;
   return (uint16_t)((u16_SensorId & 0xFFF0u) | (opt_Sensor->u8_IicBus & 0x000Fu));
}

/**
 * search for existing name entry
 */
static uint32_t m_sensor_data_search_entry(const uint16_t ou16_SensorId)
{
   uint32_t u32_Entry;

   for (u32_Entry = 0u; u32_Entry < SENSOR_NVM_DATA_MAX; u32_Entry++)
   {
      if (mat_SensorNvmDataCrc[u32_Entry].t_SensorNvmData.u16_SensorId == ou16_SensorId)
      {
         break;
      }
   }

   return u32_Entry;
}

/**
 * create default sensor data for dataset ou32_Entry
 */
static T_sensor_nvm_data * m_sensor_data_create_default(const uint32_t ou32_Entry, const T_sensor_instance * opt_Sensor)
{
   T_sensor_nvm_data * const pt_SensorNvmData = &mat_SensorNvmDataCrc[ou32_Entry].t_SensorNvmData;
   uint32_t u32_SensorNameLen;

   // create default sensor name in format "<SensorType>_BUS<x>":
   // <SensorType>=sensor type string, <x>=I2C bus number
   const char * const s_SensorType = opt_Sensor->pt_SensorDescriptor->pt_Sensor->s_Type;
   (void)sprintf(pt_SensorNvmData->ac_SensorName, "%s_BUS%u", s_SensorType, opt_Sensor->u8_IicBus);
   u32_SensorNameLen = strnlen(pt_SensorNvmData->ac_SensorName, SENSOR_NAME_LEN_MAX +1u);
   // fill unused string array data with \0
   (void)memset(&pt_SensorNvmData->ac_SensorName[u32_SensorNameLen+1u], 0, SENSOR_NAME_LEN_MAX-u32_SensorNameLen);

   // setup sensor data
   pt_SensorNvmData->u16_SensorId = m_sensor_data_get_sensor_id(opt_Sensor);
   pt_SensorNvmData->u8_SensorFlags = SENSOR_FLAGS_REMOTE_RD;

   mu32_NewData |= (1u << ou32_Entry);  // set new data flag for this entry

   return pt_SensorNvmData;
}
