/**
 * /file        menu.h
 * /brief       common menu structs and functions for terminal communication
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

#ifndef MENU_H
#define MENU_H

/**********************************************************
 * system includes
 **********************************************************/

#include "serial.h"
#include "esp_types.h"
#include "esp_err.h"

/***************************************************************************
 * defines
 ***************************************************************************/

#define MENU_MAGIC      0xFFAA5500u // menu data id magic code
#define MENU_ESC        0x1B        // ESC char

/***************************************************************************
 * data structures and types
 ***************************************************************************/

typedef void (* PR_show_menu)(void);
typedef void (* PR_exit_menu)(void);

typedef struct // common menu struct
{
   uint32_t u32_MagicCode;    // item id: sub menu or function
   PR_show_menu pr_ShowMenu;  // function shows menu mask
   PR_show_menu pr_ExitMenu;  // optional menu exit function
   uint32_t u32_MenuEntries;  // number of menu entries
   void * apv_MenuMembers[];  // pointer array of menu items (sub menu or functions)
} T_Menu;

/***************************************************************************
 * prototypes
 ***************************************************************************/

void menu_entry(const T_Menu * opt_Menu);
void set_cursor(const uint32_t ou32_Column, const uint32_t ou32_Line);
void get_cursor(uint32_t * const opu32_Column, uint32_t * const opu32_Line);
void clr_scr(void);
uint8_t get_key(void);
uint8_t get_key_nowait(void);
esp_err_t get_string(uint8_t * const opu8_String, const uint32_t ou32_Size);
esp_err_t get_uint32(uint32_t * const opu32_IntValue, const uint32_t ou32_MaxValue, const uint32_t ou32_Base);
esp_err_t get_bcd_number(uint8_t * const opu8_Data, uint32_t ou32_Digits);

#endif /* MENU_H */
