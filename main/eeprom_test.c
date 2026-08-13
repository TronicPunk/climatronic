//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  eeprom_test.c
   \brief EEPROM test functions
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "eeprom_test.h"
#include "iic_eeprom.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static void m_menu_eeprom(void);
static int32_t m_eeprom_read(void);
static int32_t m_eeprom_clear(void);
static int32_t m_eeprom_write_pattern(const uint8_t ou8_Pattern);


/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

const T_Menu gt_MenuEepromTest =
{
   MENU_MAGIC,    // menu structure identification
   m_menu_eeprom, // show menu function
   NULL,          // menu exit function
   2,             // number of menu entries
   {
      (void *)&m_eeprom_read,        // dump eeprom data
      (void *)&m_eeprom_clear,       // clear EEPROM (fill with 0xFF)
   }
};


/* -- Implementation ------------------------------------------------------------------------------------------------ */

static void m_menu_eeprom(void)
{
   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" EEPROM test functions\r\n");
   printf("------------------------------------------\r\n\n");
   printf(" [1]  Read EEPROM\r\n");
   printf(" [2]  Clear EEPROM (fill with 0xFF)\r\n\n");
   printf("[ESC] return to previous menu\r\n");
}

//-----------------------------------------------------------------------------
/*!
   \brief    read and dump EEPROM data

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_eeprom_read(void)
{
   const uint32_t u32_DumpLines = 16u;
   const uint32_t u32_DumpLineSize = 16u;
   const uint32_t u32_DumpSize = u32_DumpLines * u32_DumpLineSize;
   uint8_t u8_Key = 0u;

   clr_scr();
   printf("-------------------------------------------------------\r\n");
   printf(" Read EEPROM data\r\n");
   printf("-------------------------------------------------------\r\n\n");

   serial_clear_rx_buf(); // clear RX buffer

   if (iic_eeprom_is_ready() == ESP_OK)
   {
      uint8_t au8_EepromBuffer[u32_DumpSize];
      uint32_t u32_BufferAddr = 0u;

      printf(" [ESC] Terminate, other key = next 256byte\r\n\n");

      while (u8_Key != 0x1Bu)
      {
         // read memory
         esp_err_t t_Err = iic_eeprom_read(u32_BufferAddr, u32_DumpSize, au8_EepromBuffer);
         // dump memory
         set_cursor(0u, 7u);
         for (uint32_t u32_Line = 0u; u32_Line < u32_DumpLines; u32_Line++)
         {
            const uint32_t u32_LineOffset = u32_Line * u32_DumpLineSize;

            printf("0x%04lX: ", u32_BufferAddr + u32_LineOffset);
            if (t_Err == ESP_OK)
            {
               for (uint32_t u32_Idx = 0u; u32_Idx < u32_DumpLineSize; u32_Idx++)
               {
                  printf("%02X ", au8_EepromBuffer[u32_LineOffset + u32_Idx]);
               }
               printf("\r\n");
            }
            else
            {
               printf("EEPROM read error                    \r\n");
            }
         }

         u32_BufferAddr += u32_DumpSize;
         if (u32_BufferAddr >= IIC_EEPROM_SIZE)
         {
            u32_BufferAddr = 0u;
         }

         u8_Key = get_key();
      }
   }
   else
   {
      printf(" EEPROM not ready, press any key...\r\n\n");
      get_key();
   }

   return 1;
}


//-----------------------------------------------------------------------------
/*!
   \brief    fill EEPROM with 0xFF data

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_eeprom_clear(void)
{
   return m_eeprom_write_pattern(0xFFuL);
}

//-----------------------------------------------------------------------------
/*!
   \brief    fill EEPROM with random data

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_eeprom_write_pattern(const uint8_t ou8_Pattern)
{
   const uint32_t u32_BufferSize = 256u;
   esp_err_t t_Err;

   clr_scr();
   printf("-------------------------------------------------------\r\n");
   printf(" Write EEPROM with 0x%02X data\r\n", ou8_Pattern);
   printf("-------------------------------------------------------\r\n\n");

   serial_clear_rx_buf(); // clear RX buffer

   if (iic_eeprom_is_ready() == ESP_OK)
   {
      uint8_t au8_EepromBuffer[u32_BufferSize];
      uint32_t u32_BufferAddr = 0u;
      uint32_t u32_Line = 4u;

      while (u32_BufferAddr < IIC_EEPROM_SIZE)
      {
         set_cursor(0u, u32_Line);
         printf(" write address 0x%04lX: ", u32_BufferAddr);
         // setup data buffer
         for (uint32_t u32_Idx=0; u32_Idx < u32_BufferSize; u32_Idx++)
         {
            au8_EepromBuffer[u32_Idx] = ou8_Pattern;
         }

         t_Err = iic_eeprom_write(u32_BufferAddr, u32_BufferSize, au8_EepromBuffer);

         if (t_Err == ESP_OK)
         {
            printf("OK");
         }
         else
         {
            printf("error %d", t_Err);
            u32_Line++;
         }

         u32_BufferAddr += u32_BufferSize;
      }

      printf("\r\n\n press any key...");
      get_key();
   }
   else
   {
      printf(" EEPROM not ready, press any key...\r\n\n");
      get_key();
   }

   return 1;
}
