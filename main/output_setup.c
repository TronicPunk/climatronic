//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  output_setup.c
   \brief output setup and test functions
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <stdio.h>
#include <string.h>
#include "output_setup.h"
#include "output.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

#define PWM_STEP     50

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static void m_output_menu(void);
static int32_t m_output_test(void);
static void m_output_test_print_channel(const uint32_t ou32_Channel, const uint8_t ou8_EditFlag);
static void m_output_test_disable_all(const uint32_t ou32_EditChannel);
static int32_t m_output_setup(void);
static void m_output_setup_print_channel(const uint32_t ou32_Channel, const T_out_channel_data * const opt_OutSetup);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

// setup output menu data structure
const T_Menu gt_MenuOutputSetup =
{
   MENU_MAGIC,       // menu structure identification
   &m_output_menu,   // show output menu
   NULL,             // menu exit function
   2,                // number of menu entries
   {
      (void *)&m_output_test,    // output test
      (void *)&m_output_setup    // output setup
   }
};


/* -- Implementation ------------------------------------------------------------------------------------------------ */

static void m_output_menu(void)
{
   // display main menu
   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" Output Test and Setup\r\n");
   printf("------------------------------------------\r\n\n");
   printf(" [1] Output Test\r\n");
   printf(" [2] Output Setup\r\n\n");
   printf("[ESC] return to previous menu\r\n");
}

//-----------------------------------------------------------------------------
/*!
   \brief    set outputs manually

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_output_test(void)
{
   uint32_t u32_EditChannel = 0;
   int32_t s32_Pwm;
   uint8_t u8_Key = 0u;

   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" manual output test\r\n");
   printf("------------------------------------------\r\n");
   m_output_test_disable_all(u32_EditChannel);
   printf("\n------------------------------------------------------------\r\n");
   printf(" [1] select out channel-\r\n");
   printf(" [2] select out channel+\r\n");
   printf(" [-] set value-\r\n");
   printf(" [+] set value+\r\n");
   printf(" [5] disable all outputs\r\n\n");
   printf("[ESC] terminate test");

   serial_clear_rx_buf(); // clear RX buffer

   while (u8_Key != 0x1Bu)
   {
      if (u8_Key == '1')
      {
         m_output_test_print_channel(u32_EditChannel, false);
         u32_EditChannel = (u32_EditChannel > 0u) ? (u32_EditChannel - 1u) : (OUTPUT_NUM_CHANNEL_MAX -1u);
      }
      if (u8_Key == '2')
      {
         m_output_test_print_channel(u32_EditChannel, false);
         u32_EditChannel++;
         if (u32_EditChannel >= OUTPUT_NUM_CHANNEL_MAX)
         {
            u32_EditChannel = 0u;
         }
      }
      if (u8_Key == '-')
      {
         output_get_pwm(u32_EditChannel, &s32_Pwm);
         s32_Pwm = (s32_Pwm >= PWM_STEP) ? (s32_Pwm - PWM_STEP) : OUTPUT_PWM_MAX;
         output_set_pwm(u32_EditChannel, s32_Pwm);
      }
      if (u8_Key == '+')
      {
         output_get_pwm(u32_EditChannel, &s32_Pwm);
         s32_Pwm += PWM_STEP;
         if (s32_Pwm > OUTPUT_PWM_MAX)
         {
            s32_Pwm = 0;
         }
         output_set_pwm(u32_EditChannel, s32_Pwm);
      }
      if (u8_Key == '5')
      {
         m_output_test_disable_all(u32_EditChannel);
      }

      m_output_test_print_channel(u32_EditChannel, true);

      u8_Key = get_key();
   }

   // disable all outputs
   m_output_test_disable_all(u32_EditChannel);

   return 1;
}

static void m_output_test_print_channel(const uint32_t ou32_Channel, const uint8_t ou8_EditFlag)
{
   int32_t s32_Pwm;
   static const char* as_EditFlag[2] = { "   " , " * " };
   static const char* as_OutType[4] = { "INVALID" , "DIGITAL", "PWM 2K ", "PWM-AC " };
   T_out_channel_data t_OutSetup;
   uint8_t u8_FlagIdx = (ou8_EditFlag >= 1u) ? 1u : 0u;

   output_get_setup(ou32_Channel, &t_OutSetup);
   output_get_pwm(ou32_Channel, &s32_Pwm);
   set_cursor(0u, ou32_Channel + 5u);
   printf("%sOUTPUT%lu = %4ld %% [type = %s, def = %4u, name = %s]\r\n", as_EditFlag[u8_FlagIdx], ou32_Channel+1u, (s32_Pwm + 5)/10,
          as_OutType[t_OutSetup.u8_OutType], t_OutSetup.u16_OutValueDef, t_OutSetup.ac_OutName);
}

static void m_output_test_disable_all(const uint32_t ou32_EditChannel)
{
   for (uint32_t u32_Channel = 0u; u32_Channel < OUTPUT_NUM_CHANNEL_MAX; u32_Channel++)
   {
      uint8_t u8_EditFlag = (u32_Channel == ou32_EditChannel) ? 1u : 0u;
      output_set_pwm(u32_Channel, 0);
      m_output_test_print_channel(u32_Channel, u8_EditFlag);
   }
}

//-----------------------------------------------------------------------------
/*!
   \brief    set outputs manually

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
static int32_t m_output_setup(void)
{
   T_out_channel_data t_OutSetup;
   T_out_channel_data t_OutSetupEdit;
   uint32_t u32_EditChannel = 0;
   uint8_t u8_Key = 0u;

   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" output setup (NVM data)\r\n");
   printf("------------------------------------------\r\n");
   printf(" [1] select out channel-\r\n");
   printf(" [2] select out channel+\r\n");
   printf(" [3] set type\r\n");
   printf(" [-] default set value- [shift -5%% steps]\r\n");
   printf(" [+] default set value+ [shift +5%% steps]\r\n");
   printf(" [6] edit name\r\n");
   printf("[ESC] terminate setup");

   serial_clear_rx_buf(); // clear RX buffer

   output_data_load(u32_EditChannel, &t_OutSetup);
   t_OutSetupEdit = t_OutSetup;
   m_output_setup_print_channel(u32_EditChannel, &t_OutSetup);

   while (u8_Key != 0x1Bu)
   {
      bool q_SetupModified = false;

      if ((u8_Key == '1') || (u8_Key == '2'))
      {
         if (memcmp(&t_OutSetupEdit, &t_OutSetup, sizeof(T_out_channel_data)) != 0)
         {
            output_data_store(u32_EditChannel, &t_OutSetupEdit);
         }
         if (u8_Key == '1')
         {
            u32_EditChannel = (u32_EditChannel > 0u) ? (u32_EditChannel - 1u) : (OUTPUT_NUM_CHANNEL_MAX -1u);
         }
         else
         {
            u32_EditChannel++;
            if (u32_EditChannel >= OUTPUT_NUM_CHANNEL_MAX)
            {
               u32_EditChannel = 0u;
            }
         }
         output_data_load(u32_EditChannel, &t_OutSetup);
         t_OutSetupEdit = t_OutSetup;
         q_SetupModified = true;
      }
      if (u8_Key == '3')
      {
         t_OutSetupEdit.u8_OutType++;
         if (t_OutSetupEdit.u8_OutType >= OUTPUT_TYPE_MAX)
         {
            t_OutSetupEdit.u8_OutType = OUTPUT_TYPE_DIGITAL;
         }
         q_SetupModified = true;
      }
      if (u8_Key == '-')
      {
         t_OutSetupEdit.u16_OutValueDef = (t_OutSetupEdit.u16_OutValueDef > OUTPUT_PWM_MIN) ? (t_OutSetupEdit.u16_OutValueDef -1u) : OUTPUT_PWM_MAX;
         q_SetupModified = true;
      }
      if (u8_Key == '_')
      {
         t_OutSetupEdit.u16_OutValueDef = (t_OutSetupEdit.u16_OutValueDef >= PWM_STEP) ? (t_OutSetupEdit.u16_OutValueDef - PWM_STEP) : OUTPUT_PWM_MAX;
         q_SetupModified = true;
      }
      if ((u8_Key == '+') || (u8_Key == '*'))
      {
         if (u8_Key == '+')
         {
            t_OutSetupEdit.u16_OutValueDef++;
         }
         else
         {
            t_OutSetupEdit.u16_OutValueDef += PWM_STEP;
         }
         if (t_OutSetupEdit.u16_OutValueDef >= OUTPUT_PWM_MAX)
         {
            t_OutSetupEdit.u16_OutValueDef = OUTPUT_PWM_MIN;
         }
         q_SetupModified = true;
      }
      if (u8_Key == '6')
      {
         esp_err_t t_Err;
         char ac_OutName[OUTPUT_NAME_MAX+1];
         memset(ac_OutName, ' ', OUTPUT_NAME_MAX);
         ac_OutName[OUTPUT_NAME_MAX] = 0;
         set_cursor(50u, 14u);
         printf("%s",ac_OutName);
         set_cursor(50u, 14u);
         t_Err = get_string((uint8_t *)ac_OutName, OUTPUT_NAME_MAX);
         if (t_Err == ESP_OK)
         {
            strcpy(t_OutSetupEdit.ac_OutName, ac_OutName);
         }
         q_SetupModified = true;
      }

      if (q_SetupModified)
      {
         m_output_setup_print_channel(u32_EditChannel, &t_OutSetupEdit);
      }

      u8_Key = get_key();
   }

   if (memcmp(&t_OutSetupEdit, &t_OutSetup, sizeof(T_out_channel_data)) != 0)
   {
      output_data_store(u32_EditChannel, &t_OutSetupEdit);
   }

   return 1;
}

static void m_output_setup_print_channel(const uint32_t ou32_Channel, const T_out_channel_data * const opt_OutSetup)
{
   static const char* as_OutType[4] = { "INVALID" , "DIGITAL", "PWM 2K ", "PWM-AC " };
   char ac_OutName[OUTPUT_NAME_MAX+1];
   const uint32_t u32_NameLen = strlen(opt_OutSetup->ac_OutName);
   const uint32_t u32_RemainingSpace = OUTPUT_NAME_MAX - u32_NameLen;
   strcpy(ac_OutName, opt_OutSetup->ac_OutName);
   if (u32_RemainingSpace > 0u)
   {
      memset(&ac_OutName[u32_NameLen], ' ', u32_RemainingSpace);
      ac_OutName[OUTPUT_NAME_MAX] = 0;
   }

   set_cursor(0u, 14u);
   printf("OUTPUT%lu type = %s, def value = %4u, name = %s", ou32_Channel+1u, as_OutType[opt_OutSetup->u8_OutType],
          opt_OutSetup->u16_OutValueDef, ac_OutName);
}
