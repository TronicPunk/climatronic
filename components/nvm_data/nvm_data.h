//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  nvm_data.h
   \brief provide functions for NVM data access
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef NVM_DATA_H
#define NVM_DATA_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "nvm_data_map.h"
#include "esp_types.h"
#include "esp_err.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */
// note: we expect a 32Bit CRC at the data start address opv_Data
//       ou32_Size is the data size including the 32bit CRC
esp_err_t nvm_data_load(const uint32_t ou32_Address, const uint32_t ou32_Size, void * const opv_Data);
esp_err_t nvm_data_store(const uint32_t ou32_Address, const uint32_t ou32_Size, const void * const opv_Data);

/* -- Implementation ------------------------------------------------------------------------------------------------ */

#endif /* NVM_DATA_H */
