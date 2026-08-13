//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  iic_eeprom.c
   \brief provide functions for IIC EEPROM access
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "iic_eeprom.h"
#include "iic.h"
#include "esp_timer.h"
#include <string.h>


/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define IIC_EEPROM_ADDR             0x50u       // IIC eeprom device address
#define IIC_EEPROM_SCL_FREQ         400000u     // IIC eeprom SCL frequency
#define IIC_EEPROM_SCL_TIMEOUT_US   10000u      // IIC eeprom SCL timeout in us
#define IIC_EEPROM_TIMEOUT_MS       50u         // IIC eeprom transaction timeout in ms
#define IIC_EEPROM_ADDR_SIZE        2u          // IIC eeprom with 2 byte addressing mode
#define IIC_EEPROM_WRITE_TIMEOUT_US 20000u      // IIC eeprom write timeout 20ms

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */
static bool m_iic_eeprom_ready(void);
static esp_err_t m_iic_eeprom_wait_busy(const uint32_t ou32_TimeoutUs);
static esp_err_t m_iic_eeprom_check_param(const uint32_t ou32_Address, const uint32_t ou32_Size, const uint8_t * const opu8_Data);
static esp_err_t m_iic_eeprom_write_page(const uint32_t ou32_Address, const uint32_t ou32_Size, const uint8_t * const opu8_Data);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */
static i2c_master_bus_handle_t mt_IicBusHandle = NULL;
static i2c_master_dev_handle_t mt_IicDevHandle_Eeprom = NULL;

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * check if EEPROM device is ready
 */
static bool m_iic_eeprom_ready(void)
{
   const esp_err_t t_Err = i2c_master_probe(mt_IicBusHandle, IIC_EEPROM_ADDR, IIC_EEPROM_TIMEOUT_MS);

   return (t_Err == ESP_OK) ? true : false;
}

/**
 * low level EEPROM wait busy with timeout
 */
static esp_err_t m_iic_eeprom_wait_busy(const uint32_t ou32_TimeoutUs)
{
   const uint64_t u64_TimeoutTime = (uint64_t)ou32_TimeoutUs + esp_timer_get_time();
   bool q_EepromReady;
   bool q_Timeout;

   do
   {
      q_EepromReady = m_iic_eeprom_ready();
      q_Timeout = (esp_timer_get_time() > u64_TimeoutTime) ? true : false;
   } while (!q_EepromReady && !q_Timeout);

   return (q_EepromReady) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/**
 * check input parameters
 */
static esp_err_t m_iic_eeprom_check_param(const uint32_t ou32_Address, const uint32_t ou32_Size, const uint8_t * const opu8_Data)
{
   esp_err_t t_Err = ESP_ERR_INVALID_STATE;

   if (mt_IicDevHandle_Eeprom != NULL)
   {
      t_Err = ((ou32_Address < IIC_EEPROM_SIZE) && (ou32_Size <= IIC_EEPROM_SIZE) &&
               ((ou32_Address+ou32_Size) <= IIC_EEPROM_SIZE) && (opu8_Data != 0u)) ? ESP_OK : ESP_ERR_INVALID_ARG;
   }

   return t_Err;
}

/**
 * register the EEPROM device to the ESP32 IIC master interface
 */
esp_err_t iic_eeprom_init(void)
{
   static const i2c_device_config_t t_IicDev_Eeprom =
   {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = IIC_EEPROM_ADDR,
      .scl_speed_hz = IIC_EEPROM_SCL_FREQ,
      .scl_wait_us = IIC_EEPROM_SCL_TIMEOUT_US,
      .flags.disable_ack_check = false
   };
   esp_err_t t_Err;
   mt_IicBusHandle = iic_get_bus_handle();

   if (mt_IicBusHandle == NULL)
   {
      t_Err = ESP_FAIL;
   }
   else
   {
      // register the IIC eeprom device
      t_Err = i2c_master_bus_add_device(mt_IicBusHandle, &t_IicDev_Eeprom, &mt_IicDevHandle_Eeprom);
      ESP_ERROR_CHECK(t_Err);
   }

   return t_Err;
}

/**
 * check if EEPROM device is ready
 */
esp_err_t iic_eeprom_is_ready(void)
{
    esp_err_t t_Err = (mt_IicDevHandle_Eeprom != NULL) ? ESP_OK : ESP_ERR_INVALID_STATE;

   if (t_Err == ESP_OK)
   {
      if (!m_iic_eeprom_ready())
      {
         t_Err = ESP_FAIL;
      }
   }

   return t_Err;
}


/**
 * read from the EEPROM device
 */
esp_err_t iic_eeprom_read(const uint32_t ou32_Address, const uint32_t ou32_Size, uint8_t * const opu8_Data)
{
   esp_err_t t_Err = m_iic_eeprom_check_param(ou32_Address, ou32_Size, opu8_Data);

   if (t_Err == ESP_OK)
   {
      esp_err_t t_Err = m_iic_eeprom_wait_busy(IIC_EEPROM_WRITE_TIMEOUT_US);
      if (t_Err == ESP_OK)
      {
         uint8_t au8_Address[IIC_EEPROM_ADDR_SIZE];

         // setup EEPROM memory address in 16bit big indian format
         au8_Address[0] = (uint8_t)((ou32_Address >> 8u) & 0xFFu);
         au8_Address[1] = (uint8_t)(ou32_Address & 0xFFu);

         t_Err = i2c_master_transmit_receive(mt_IicDevHandle_Eeprom, au8_Address, IIC_EEPROM_ADDR_SIZE,
                                             opu8_Data, ou32_Size, IIC_EEPROM_TIMEOUT_MS);
         t_Err = (t_Err != ESP_OK) ? ESP_FAIL : t_Err;
      }
   }

   return t_Err;
}


/**
 * write into the EEPROM device
 */
esp_err_t iic_eeprom_write(const uint32_t ou32_Address, const uint32_t ou32_Size, const uint8_t * const opu8_Data)
{
   esp_err_t t_Err = m_iic_eeprom_check_param(ou32_Address, ou32_Size, opu8_Data);

   if (t_Err == ESP_OK)
   {
      uint32_t u32_Addr = ou32_Address;
      const uint8_t * pu8_Data = opu8_Data;
      uint32_t u32_Size = ou32_Size;

      while ((u32_Size != 0u) && (t_Err == ESP_OK))
      {
         uint32_t u32_ByteInPage = ((uint32_t)(~u32_Addr) & (IIC_EEPROM_PAGE_SIZE-1u)) +1u;
         uint32_t u32_ProgSize = (u32_Size > u32_ByteInPage) ? u32_ByteInPage : u32_Size;

         t_Err = m_iic_eeprom_write_page(u32_Addr, u32_ProgSize, pu8_Data);

         u32_Addr += u32_ProgSize;
         u32_Size -= u32_ProgSize;
         pu8_Data = &pu8_Data[u32_ProgSize];
      }
   }

   return t_Err;
}

/**
 * low level EEPROM page write
 */
static esp_err_t m_iic_eeprom_write_page(const uint32_t ou32_Address, const uint32_t ou32_Size, const uint8_t * const opu8_Data)
{
   esp_err_t t_Err = m_iic_eeprom_wait_busy(IIC_EEPROM_WRITE_TIMEOUT_US);

   if (t_Err == ESP_OK)
   {
      uint8_t au8_WritePageBuffer[IIC_EEPROM_PAGE_SIZE + IIC_EEPROM_ADDR_SIZE];

      // setup EEPROM memory address in 16bit big indian format
      au8_WritePageBuffer[0] = (uint8_t)((ou32_Address >> 8u) & 0xFFu);
      au8_WritePageBuffer[1] = (uint8_t)(ou32_Address & 0xFFu);
      // copy data into write page buffer
      memcpy(&au8_WritePageBuffer[2], opu8_Data, ou32_Size);

      t_Err = i2c_master_transmit(mt_IicDevHandle_Eeprom, au8_WritePageBuffer, IIC_EEPROM_ADDR_SIZE + ou32_Size, IIC_EEPROM_TIMEOUT_MS);
      t_Err = (t_Err != ESP_OK) ? ESP_FAIL : t_Err;
   }

   return t_Err;
}
