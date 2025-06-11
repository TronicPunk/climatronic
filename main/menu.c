/**
 * /file        menu.c
 * /brief       common menu structs and functions
 *
 * /author      Ulrich Herb
 * /date        09.02.2006
 * /version     1.00
 *
 *
 * /par Specifications:
 *
 * /par History:
 * 09.02.2006:  Herb, file created \n
 *****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "menu.h"
#include "timer.h"


/***************************************************************************
 * defines
 ***************************************************************************/
#define TASK_DELAY_MS      50

/***************************************************************************
 * types
 ***************************************************************************/
typedef int32_t (* PR_test_func)(void);

/***************************************************************************
 * global variables
 ***************************************************************************/

/***************************************************************************
 * module internal prototypes
 ***************************************************************************/

static uint8_t m_get_menu_item(const uint32_t ou32_MenuEntries);

/***************************************************************************
 * implementation
 ***************************************************************************/

/***************************************************************************
 * menu function
 ***************************************************************************/

void menu_entry(const T_Menu * opt_Menu)
{
   uint8_t u8_Key = 0u;

   if (opt_Menu->pr_ShowMenu != NULL) // pointer to show menu available?
   {
      while (u8_Key != MENU_ESC)
      {
         (*opt_Menu->pr_ShowMenu)(); // show menu

         // get user input from RS232
         u8_Key = m_get_menu_item(opt_Menu->u32_MenuEntries);

         if (u8_Key < opt_Menu->u32_MenuEntries) // valid menu item?
         {
            if (opt_Menu->apv_MenuMembers[u8_Key] != NULL)
            {           
               void * const pv_FuncMenu = opt_Menu->apv_MenuMembers[u8_Key];

               // check if pv_FuncMenu points to a further menu structure
               if (*((uint32_t*)pv_FuncMenu) == MENU_MAGIC)
               {
                  menu_entry(pv_FuncMenu);
               }
               else
               { 
                  PR_test_func pr_test_func = pv_FuncMenu;

                  int32_t s32_RetVal = (*pr_test_func)();

                  if (s32_RetVal == 0)
                  {
                     fputs("\r\npress any key...", stdout);
                     get_key();
                  }
               }
            }
         }
      }

      if (opt_Menu->pr_ExitMenu != NULL) // pointer to exit function available?
      {
         (*opt_Menu->pr_ExitMenu)(); // call exit function
      }
   }
}

/**
 * \brief   set terminal cursor at position X/Y
 *
 * \par History:
 * 10.07.2007: Herb, function created \n
 *****************************************************************************/
void set_cursor(const uint32_t ou32_Column, const uint32_t ou32_Line)
{
   fprintf(stdout, "[%u;%uH", (unsigned int)ou32_Line, (unsigned int)ou32_Column);
}

/**
 * \brief   get terminal cursor position
 *****************************************************************************/
void get_cursor(uint32_t * const opu32_Column,  uint32_t * const opu32_Line)
{
   char acn_Buffer[20];
   uint8_t u8_Num = 0;
   char  * pcn_LineStart, * pcn_LineEnd, * pcn_ColumnStart, * pcn_ColumnEnd;
   int sn_Line, sn_Column;

   serial_clear_rx_buf();
   fputs("[6n", stdout);
   serial_flush_tx_buf();

   acn_Buffer[u8_Num] = 0;
   while (true)
   {
      int cn_Char = fgetc(stdin);

      if (cn_Char != EOF)
      {
         acn_Buffer[u8_Num] = (char)cn_Char;

         if (acn_Buffer[u8_Num] != 'R')
         {
            u8_Num++;
         }
         else
         {
            break;
         }
         acn_Buffer[u8_Num] = 0;
      }
      else
      {
         // if no char is available wait and release CPU time for a few milli seconds
         delay_us(TASK_DELAY_MS * 1000u);
      }
   }

   pcn_LineStart = acn_Buffer;

   while (*pcn_LineStart++ != '[')
   {
      ;
   }
   pcn_LineEnd = pcn_LineStart;
   while (*++pcn_LineEnd != ';')
   {
      ;
   }
   *pcn_LineEnd = 0;
   pcn_ColumnStart = pcn_LineEnd;
   pcn_ColumnEnd = ++pcn_ColumnStart;
   while (*++pcn_ColumnEnd != 'R')
   {
      ;
   }
   *pcn_ColumnEnd = 0;

   sscanf(pcn_LineStart, "%d", &sn_Line);
   sscanf(pcn_ColumnStart, "%d", &sn_Column);

   *opu32_Line = sn_Line;
   *opu32_Column = sn_Column;
}

/**
 * \brief   "clear screen" routine
 *
 * \par History:
 * 28.04.2008: Baldauf, function created \n
 *****************************************************************************/
void clr_scr(void)
{
   fputs("[2J", stdout);   //clear screen
   fputs("[1;1H", stdout); // cursor to Pos. 1 1
}

/***************************************************************************
 * get_key: wait for char from RS232 interface
 ***************************************************************************/
uint8_t get_key(void)
{
   int Key;

   serial_flush_tx_buf();  // send all characters befor we wait for any input
   serial_clear_rx_buf();  // clear RX buffer

   do
   {
      Key = fgetc(stdin);
      
      // if no char is available wait and release CPU time for a few milli seconds
      if (Key == EOF)      
      {
         delay_us(TASK_DELAY_MS * 1000u);
      }
   }
   while (Key == EOF);
   
   return (uint8_t)Key;
}

/***************************************************************************
 * get_key_nowait: try to read a char form RS232 interface, return 0
 * if none available
 ***************************************************************************/
uint8_t get_key_nowait(void)
{
   int Key = fgetc(stdin);

   if (Key == EOF)
   {
      Key = 0;
   }

   return (uint8_t)Key;
}

/**
 * \brief   read unterminated string from RS232 interface
 *
 * \par History:
 * 29.01.2007: Herb, function created \n
 *****************************************************************************/
esp_err_t get_string(uint8_t * const opu8_String, uint32_t ou32_Size)
{
   esp_err_t t_Err = ESP_FAIL;
   uint32_t i = 0u;
   uint8_t u8_Char;

   while (true)
   {
      do // get digit
      {
         u8_Char = get_key();
      }
      while ((isgraph(u8_Char) == false) && (u8_Char != '\r') && (u8_Char != '\b') && (u8_Char != ''));

      if (u8_Char == '')
      {
         break;
      }
      else if (u8_Char == '\r')
      {
         opu8_String[i] = '\0'; // terminate string < ou32_Size
         t_Err = ESP_OK;
         break;
      }
      else if (u8_Char == '\b')
      {
         if (i > 0)
         {
            i--;
            fwrite("\b \b", 1, 3, stdout); // echo input digit
         }
      }
      else
      {
         if (i < ou32_Size)
         {
            opu8_String[i++] = u8_Char;
            fputc(u8_Char, stdout); // echo input digit
         }
      }
   }

   return t_Err;
}

/**
 * \brief   read unsigned integer value from RS232 interface
 *
 * \par History:
 * 29.01.2007: Herb, function created \n
 *****************************************************************************/

 esp_err_t get_uint32(uint32_t * const opu32_IntValue, const uint32_t ou32_MaxValue, const uint32_t ou32_Base)
{
   char ac_String[11];
   uint32_t u32_Index = 0, u32_Value = 0;
   uint8_t u8_Char;
   esp_err_t t_Err = ESP_FAIL;

   while (t_Err == ESP_FAIL)
   {
      uint32_t u32_key_valid;

      do // get hex digit
      {
         u8_Char = get_key();

         if (ou32_Base == 10)
         {
            u32_key_valid = isdigit(u8_Char);
         }
         else
         {
            u32_key_valid  = isxdigit(u8_Char);
            u32_key_valid |= ((u8_Char == 'x') || (u8_Char == 'X'));
         }
      }
      while ((u32_key_valid == false) && (u8_Char != '\r') && (u8_Char != '\b') && (u8_Char != ''));

      if (u8_Char == '') // ESC
      {
         break;
      }
      else if (u8_Char == '\r') // enter
      {
         char * pc_RestStr;

         ac_String[u32_Index] = '\0';

         u32_Value = strtoul(ac_String, &pc_RestStr, ou32_Base);

         if ((*pc_RestStr == '\0') && (u32_Value <= ou32_MaxValue))
         {
            *opu32_IntValue = u32_Value;
            t_Err = ESP_OK;
         }
         else
         {
            t_Err = ESP_ERR_INVALID_SIZE;
         }
      }
      else if (u8_Char == '\b') // backspace
      {
         if (u32_Index > 0)
         {
            u32_Index--;
            fwrite("\b \b", 1, 3, stdout); // echo input digit
         }
      }
      else
      {
         if (u32_Index < 10)
         {
            ac_String[u32_Index++] = u8_Char;
            fputc(u8_Char, stdout); // echo input digit
         }
      }
   }

   return t_Err;
}

/**
 * \brief   read digits from RS232 input
 *
 * \par History:
 * 29.01.2007: Herb, function created \n
 *****************************************************************************/

esp_err_t get_bcd_number(uint8_t * const opu8_Data, uint32_t ou32_Digits)
{
   uint8_t u8_Char, u8_Digit;
   uint32_t u32_MinIndex = 0u, u32_Index = 0u;
   esp_err_t t_Err = ESP_FAIL;

   if ((ou32_Digits & 0x1u) != 0u)
   {
      u32_Index = 1u;
      u32_MinIndex = 1u;
      ou32_Digits += 1u;
      opu8_Data[0] = 0u;
   }

   while (t_Err == ESP_FAIL)
   {
      do // get hex digit
      {
         u8_Char = get_key();
      }
      while ((isxdigit(u8_Char) == false) && (u8_Char != '\b') && (u8_Char != ''));

      if (u8_Char == '') // ESC
      {
         break;
      }
      else if (u8_Char == '\b') // backspace?
      {
         if (u32_Index > u32_MinIndex)
         {
            u32_Index--;
            fwrite("\b \b", 1, 3, stdout); // echo input digit
         }
      }
      else
      {
         if ((u8_Char >= '0') && (u8_Char <= '9'))
         {
            u8_Digit = u8_Char - '0'; // convert ASCII code '0'..'9'
         }
         else if ((u8_Char >= 'a') && (u8_Char <= 'f'))
         {
            u8_Digit = u8_Char - 'a' + 10u; // convert ASCII code 'a'..'f'
         }
         else if ((u8_Char >= 'A') && (u8_Char <= 'F'))
         {
            u8_Digit = u8_Char - 'A' + 10u; // convert ASCII code 'A'..'F'
         }
         else
         {
            u8_Digit = 0u; // unknown value, return zero!
         }

         if ((u32_Index & 0x1u) != 0u)
         {
            // set low nibble
            opu8_Data[u32_Index >> 1] &= 0xF0u;
            opu8_Data[u32_Index >> 1] |= u8_Digit;
         }
         else
         {
            // set high nibble
            opu8_Data[u32_Index >> 1] &= 0x0Fu;
            opu8_Data[u32_Index >> 1] |= (u8_Digit << 4);
         }

         fputc(u8_Char, stdout); // echo input digit

         u32_Index++;
      }

      if (u32_Index >= ou32_Digits)
      {
         t_Err = ESP_OK;
      }
   }

   return t_Err;
}

/**
 * \brief   wait until all chars in tx buffer are transmitted
 *
 * \par History:
 * 15.02.2007: Herb, function created \n
 *****************************************************************************/
void wait_until_tx_buffer_empty(void)
{
   fflush(stdout);
}

/***************************************************************************
 * m_get_menu_item: wait for menu item from RS232 interface
 ***************************************************************************/
static uint8_t m_get_menu_item(const uint32_t ou32_MenuEntries)
{
   uint8_t u8_Key = 0xFFu;

   do
   {
      u8_Key = get_key();

      if (u8_Key != MENU_ESC) // input is not ESC key?
      {
         if ((u8_Key >= 0x31u) && (u8_Key <= 0x39u)) // valid menu item (1..9) ?
         {
            u8_Key = u8_Key - 0x31u; // create integer index 0..8

            if (u8_Key >= ou32_MenuEntries) // menu item not valid?
            {
               u8_Key = 0xFFu; // yes -> invalidate input
            }
         }
         else if ((u8_Key >= 'a') && (u8_Key <= 'z')) // valid menu item (a..z) ?
         {
            u8_Key = u8_Key - 'a' + 9; // create integer index 9..34

            if (u8_Key >= ou32_MenuEntries) // menu item not valid?
            {
               u8_Key = 0xFFu; // yes -> invalidate input
            }
         }
         else
         {
            u8_Key = 0xFFu; // input not valid!
         }
      }
   }
   while (u8_Key == 0xFFu); // wait for valid char...

   return u8_Key;
}
