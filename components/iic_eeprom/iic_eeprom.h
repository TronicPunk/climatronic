//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  iic_eeprom.h
   \brief provide functions for IIC eeprom device
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef IIC_EEPROM_H
#define IIC_EEPROM_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "esp_types.h"
#include "esp_err.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

// device parameters for M24C64-DFMN6
#define IIC_EEPROM_SIZE             8192u    // memory size 8KB
#define IIC_EEPROM_PAGE_SIZE        32u      // memory page size is 32byte

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

esp_err_t iic_eeprom_init(void);
esp_err_t iic_eeprom_is_ready(void);
esp_err_t iic_eeprom_read(const uint32_t ou32_Address, const uint32_t ou32_Size, uint8_t * const opu8_Data);
esp_err_t iic_eeprom_write(const uint32_t ou32_Address, const uint32_t ou32_Size, const uint8_t * const opu8_Data);

/* -- Implementation ------------------------------------------------------------------------------------------------ */

#endif /* IIC_EEPROM_H */
