//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  nvm_data.c
   \brief provide functions for NVM data access
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "nvm_data.h"
#include "iic_eeprom.h"
#include "esp_crc.h"
#include <string.h>


/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define NVM_DATA_CRC_OFFS        sizeof(uint32_t)

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */
static uint32_t m_nvm_data_get_crc32(const uint8_t * const opu8_Data);
static void     m_nvm_data_set_crc32(uint8_t * const opu8_Data, const uint32_t ou32_CRC32);

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * Load data from NVM address ou32_Address and also from NVM mirror address space.
 * Check CRC for both data sets. If both data sets are invalid return with ESP_ERR_INVALID_CRC.
 * If one data set is invalid then repair it with the remaining valid data set.
 *
 * @param ou32_Address     NVM data address (see nvm_data_map.h)
 * @param ou32_Size        NVM data size (including 32Bit CRC)
 * @param opv_Data         return the NVM data on success
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALID_ARG: Parameter ou32_Address or ou32_Size out of range
 *  - ESP_ERR_INVALID_CRC: CRC data error, no data returned
 */
esp_err_t nvm_data_load(const uint32_t ou32_Address, const uint32_t ou32_Size, void * const opv_Data)
{
   esp_err_t t_Err;
   esp_err_t t_ErrRd;
   esp_err_t t_ErrRdMirror;
   uint8_t * const pu8_Data = (uint8_t*)opv_Data;
   uint8_t au8_DataMirror[ou32_Size];

   // preset data to invalidate it
   *au8_DataMirror = 0x55u;
   *pu8_Data = 0xAAu;
   // read data
   t_ErrRd = iic_eeprom_read(ou32_Address, ou32_Size, pu8_Data);
   t_ErrRdMirror = iic_eeprom_read(ou32_Address + NVM_DATA_MIRROR_ADDR, ou32_Size, au8_DataMirror);
   if ((t_ErrRd == ESP_ERR_INVALID_ARG) || (t_ErrRdMirror == ESP_ERR_INVALID_ARG))
   {
      t_Err = ESP_ERR_INVALID_ARG;
   }
   else
   {
      uint32_t u32_CRC32;
      uint32_t u32_CRC32_Mirror;
      bool q_DataValid = false;
      bool q_DataMirrorValid = false;

      // check data
      u32_CRC32 = esp_crc32_le(0u, &pu8_Data[NVM_DATA_CRC_OFFS], ou32_Size - NVM_DATA_CRC_OFFS);
      if (m_nvm_data_get_crc32(pu8_Data) == u32_CRC32)
      {
         q_DataValid = true;
      }
      u32_CRC32_Mirror = esp_crc32_le(0u, &au8_DataMirror[NVM_DATA_CRC_OFFS], ou32_Size - NVM_DATA_CRC_OFFS);
      if (m_nvm_data_get_crc32(au8_DataMirror) == u32_CRC32_Mirror)
      {
         q_DataMirrorValid = true;
      }

      if (!q_DataValid && !q_DataMirrorValid)
      {
         t_Err = ESP_ERR_INVALID_CRC;
      }
      else
      {
         if (!q_DataValid)
         {
            (void)memcpy(pu8_Data, au8_DataMirror, ou32_Size);
            (void)iic_eeprom_write(ou32_Address, ou32_Size, pu8_Data);
         }
         else if ((!q_DataMirrorValid) || (u32_CRC32 != u32_CRC32_Mirror))
         {
            (void)iic_eeprom_write(ou32_Address + NVM_DATA_MIRROR_ADDR, ou32_Size, pu8_Data);
         }
         else
         {
            // nothing to be done
         }

         t_Err = ESP_OK;
      }
   }

   return t_Err;
}


/**
 * Store data to NVM address ou32_Address and also to NVM mirror address space.
 * Calculate CRC for both data sets.
 *
 * @param ou32_Address     NVM data address (see nvm_data_map.h)
 * @param ou32_Size        NVM data size (including 32Bit CRC)
 * @param opv_Data         pointer to data to be stored
 *
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALID_ARG: Parameter ou32_Address or ou32_Size out of range
 *  - ESP_FAIL:            EEPROM RD/WR error
 */
esp_err_t nvm_data_store(const uint32_t ou32_Address, const uint32_t ou32_Size, const void * const opv_Data)
{
   esp_err_t t_Err;
   esp_err_t t_ErrWr;
   esp_err_t t_ErrWrMirror;
   uint8_t * const pu8_Data = (uint8_t*)opv_Data;
   // calculate CRC32
   uint32_t u32_CRC32 = esp_crc32_le(0u, &pu8_Data[NVM_DATA_CRC_OFFS], ou32_Size - NVM_DATA_CRC_OFFS);

   m_nvm_data_set_crc32(pu8_Data, u32_CRC32);
   t_ErrWr = iic_eeprom_write(ou32_Address, ou32_Size, pu8_Data);
   t_ErrWrMirror = iic_eeprom_write(ou32_Address + NVM_DATA_MIRROR_ADDR, ou32_Size, pu8_Data);

   if ((t_ErrWr == ESP_ERR_INVALID_ARG) || (t_ErrWrMirror == ESP_ERR_INVALID_ARG))
   {
      t_Err = ESP_ERR_INVALID_ARG;
   }
   else
   {
      t_Err = ((t_ErrWr != ESP_OK) || (t_ErrWrMirror != ESP_OK)) ? ESP_FAIL : ESP_OK;
   }

   return t_Err;
}


/**
 * Read CRC32 value from byte array in little endian format
 *
 * @param opu8_Data  pointer to byte array
 *
 * @return uint32_t  CRC32 value
 */
static uint32_t m_nvm_data_get_crc32(const uint8_t * const opu8_Data)
{
   return ((uint32_t)opu8_Data[3] << 24u) | ((uint32_t)opu8_Data[2] << 16u) | ((uint32_t)opu8_Data[1] << 8u) | opu8_Data[0];
}


/**
 * Write CRC32 value into byte array in little endian format
 *
 * @param opu8_Data     pointer to byte array
 * @param ou32_CRC32    CRC32 value
 */
static void m_nvm_data_set_crc32(uint8_t * const opu8_Data, const uint32_t ou32_CRC32)
{
   opu8_Data[0] = (uint8_t)ou32_CRC32;
   opu8_Data[1] = (uint8_t)(ou32_CRC32 >> 8u);
   opu8_Data[2] = (uint8_t)(ou32_CRC32 >> 16u);
   opu8_Data[3] = (uint8_t)(ou32_CRC32 >> 24u);
}
