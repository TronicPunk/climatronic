/*
   output configuration and setup data
   load/store output data from/to NVM
*/

/* --- Includes ------------------------------------------------------------ */
#include "output_data.h"
#include "nvm_data.h"
#include <string.h>

/* --- Defines ------------------------------------------------------------- */

/* --- Types --------------------------------------------------------------- */

// output data set
typedef struct
{
   uint32_t u32_CRC32;                       // CRC32
   T_out_channel_data t_OutChannelSetup;    // output channel setup data
} T_out_channel_data_crc;

_Static_assert(sizeof(T_out_channel_data_crc) == NVM_DATA_OUT_SIZE, "T_out_channel_data_crc must be of size NVM_DATA_OUT_SIZE byte");

/* --- Module global variables --------------------------------------------- */

static const T_out_channel_data mt_OutChannelInvalid =
{
   .u16_OutValueDef = 0u,
   .u8_OutType = OUTPUT_TYPE_INVALID,
   .ac_OutName = "OUTx INVALID"
};

static const T_out_channel_data mt_OutChannelSetupDef =
{
   .u16_OutValueDef = 0u,
   .u8_OutType = OUTPUT_TYPE_DIGITAL,
   .ac_OutName = "OUTx"
};

/* --- Module global prototypes -------------------------------------------- */

/* --- Implementation ------------------------------------------------------ */

/**
 * @brief load output channel setup data
 *
 * @param ou32_Channel           output channel, range: [0 .. OUTPUT_NUM_CHANNEL_MAX-1]
 * @param opt_OutChannelSetup    copy output channel setup data to this location
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALID_ARG: Parameter ou32_Channel out of range
 *  - ESP_ERR_INVALID_CRC: CRC data error, return default values
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t output_data_load(const uint32_t ou32_Channel, T_out_channel_data * const opt_OutChannelSetup)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;
   T_out_channel_data_crc t_OutChannelSetupCrc;
   bool q_DataValid = false;

   if (ou32_Channel < OUTPUT_NUM_CHANNEL_MAX)
   {
      const uint32_t u32_NvmAddr = NVM_DATA_OUT_ADDR + (ou32_Channel * sizeof(T_out_channel_data_crc));

      // read eeprom data
      t_Err = nvm_data_load(u32_NvmAddr, sizeof(T_out_channel_data_crc), &t_OutChannelSetupCrc);
      if (t_Err == ESP_ERR_INVALID_CRC)
      {
         esp_err_t t_ErrStore;

         t_OutChannelSetupCrc.t_OutChannelSetup = mt_OutChannelSetupDef;
         // create default output name OUT1...OUT8
         t_OutChannelSetupCrc.t_OutChannelSetup.ac_OutName[3] = (char)('1' + ou32_Channel);
         t_ErrStore = nvm_data_store(u32_NvmAddr, sizeof(T_out_channel_data_crc), &t_OutChannelSetupCrc);

         q_DataValid = (t_ErrStore == ESP_OK) ? true : false;
         t_Err = (t_ErrStore == ESP_FAIL) ? ESP_FAIL : t_Err;
      }
      else
      {
         q_DataValid = (t_Err == ESP_OK) ? true : false;
      }
   }

   if (q_DataValid)
   {
      *opt_OutChannelSetup = t_OutChannelSetupCrc.t_OutChannelSetup;
   }
   else
   {
      *opt_OutChannelSetup = mt_OutChannelInvalid;
      opt_OutChannelSetup->ac_OutName[3] = (char)('1' + ou32_Channel);
   }

   return t_Err;
}

/**
 * @brief store output channel setup data
 *
 * @param ou32_Channel           output channel, range: [0 .. OUTPUT_NUM_CHANNEL_MAX-1]
 * @param opt_OutChannelSetup    output channel setup data to be stored
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALID_ARG: Parameter ou32_Channel out of range
 *  - ESP_ERR_INVALID_SIZE: Invalid data, not stored
 *  - ESP_FAIL: EEPROM read/write error
 */
esp_err_t output_data_store(const uint32_t ou32_Channel, const T_out_channel_data * const opt_OutChannelSetup)
{
   esp_err_t t_Err = ESP_ERR_INVALID_ARG;

   if (ou32_Channel < OUTPUT_NUM_CHANNEL_MAX)
   {
      const uint32_t u32_NameLen = strnlen(opt_OutChannelSetup->ac_OutName, sizeof(opt_OutChannelSetup->ac_OutName));

      // check output channel setup data
      if ((opt_OutChannelSetup->u8_OutType < OUTPUT_TYPE_MAX) && (opt_OutChannelSetup->u16_OutValueDef <= OUTPUT_PWM_MAX) &&
          (u32_NameLen > 0u) && (u32_NameLen <= OUTPUT_NAME_MAX))
      {
         T_out_channel_data_crc t_OutChannelSetupCrc;
         T_out_channel_data * const pt_OutChannelSetup = &t_OutChannelSetupCrc.t_OutChannelSetup;
         const uint32_t u32_NvmAddr = NVM_DATA_OUT_ADDR + (ou32_Channel * sizeof(T_out_channel_data_crc));

         *pt_OutChannelSetup = *opt_OutChannelSetup;
         // fill unused name array data with \0
         (void)memset(&pt_OutChannelSetup->ac_OutName[u32_NameLen+1u], 0, OUTPUT_NAME_MAX-u32_NameLen);
         // store setup data
         t_Err = nvm_data_store(u32_NvmAddr, sizeof(T_out_channel_data_crc), &t_OutChannelSetupCrc);
      }
      else
      {
         t_Err = ESP_ERR_INVALID_SIZE;
      }
   }

   return t_Err;
}
