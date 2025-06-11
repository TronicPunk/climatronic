//----------------------------------------------------------------------------------------------------------------------
/*!
   \file
   \brief provide a timer functions like delay or timer callbacks for data process tasks   
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "timer.h"
#include "freertos/FreeRTOS.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * delay function with us resolution and RTOS awareness
 * note: for delay times > RTOS ticks make use of RTOS delay/sleep function
 */
void delay_us(const uint32_t ou32_TimeUs)
{
   const uint64_t u64_StartTime = esp_timer_get_time();
   const uint64_t u64_StopTime = u64_StartTime + ou32_TimeUs;
   const uint32_t u32_TaskTicks = ou32_TimeUs / (portTICK_PERIOD_MS * 1000u);

   if (u32_TaskTicks > 0u)
   {
      vTaskDelay(u32_TaskTicks);    // wait rough time in task ticks (e.g 10ms)
   }
   while (esp_timer_get_time() < u64_StopTime)
   {
      // wait remaining delay in exact us
   }
}
