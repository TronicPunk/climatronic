//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  sensor_setup.c
   \brief sensor test and setup functions
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <stdio.h>
#include <string.h>
#include "sensor_setup.h"
#include "sensor.h"
#include "sensor_name.h"
#include "timer.h"


/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

/* -- Implementation ------------------------------------------------------------------------------------------------ */

//-----------------------------------------------------------------------------
/*!
   \brief    read all sensor values

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
int32_t sensor_test(void)
{
   const uint16_t u16_SensorCnt = sensor_get_count();
   uint16_t u16_SensorIdx;
   T_sensor_info t_SensorInfo;
   uint8_t u8_Key = 0u;

   clr_scr();
   printf("-------------------------------------------------------\r\n");
   printf(" Show Sensor data ([ESC] terminate, [c] calibrate CO2)\r\n");
   printf("-------------------------------------------------------\r\n\n");
   printf("Sensor Type   Name                     | T [°C] | P [hPa] | Hum [%%] | CO2 [ppm] | sensor state | data age [s]\r\n");
   printf("---------------------------------------------------------------------------------------------------------------\r\n");

   for (u16_SensorIdx = 0u; u16_SensorIdx < u16_SensorCnt; u16_SensorIdx++)
   {
      (void)sensor_get_info(u16_SensorIdx, &t_SensorInfo);

      printf("%2u %s", u16_SensorIdx, t_SensorInfo.pt_Sensor->s_Type);
      set_cursor(15u, u16_SensorIdx + 7u);
      printf("%s", t_SensorInfo.s_Name);
      set_cursor(40u, u16_SensorIdx + 7u);
      printf("|  ---.- |  ----.- |  ---.-  |   -----   |  0x00000000  |\r\n");
   }

   serial_clear_rx_buf(); // clear RX buffer

   while (u8_Key != 0x1Bu)
   {
      delay_us(1000000u);

      if ((u8_Key == 'c') || (u8_Key == 'C'))
      {
         // calibrate all CO2 sensors
         for (u16_SensorIdx = 0; u16_SensorIdx < u16_SensorCnt; u16_SensorIdx++)
         {
            (void)sensor_get_info(u16_SensorIdx, &t_SensorInfo);

            if ((t_SensorInfo.pt_Sensor->u16_SensorCaps & SENSOR_CAP_CO2) != 0u)
            {
               uint32_t u32_Parameter = 400u; // calibrate CO2 sensors to 400ppm
               (void)sensor_command(u16_SensorIdx, SENSOR_COMMAND_CALIBRATE_CO2, &u32_Parameter);
            }
         }
      }

      // read data from all connected sensors
      for (u16_SensorIdx = 0; u16_SensorIdx < u16_SensorCnt; u16_SensorIdx++)
      {
         T_sensor_data t_SensorData;
         uint32_t u32_SensorState;

         (void)sensor_get_data(u16_SensorIdx, &t_SensorData);           // read sensor data
         (void)sensor_command(u16_SensorIdx, SENSOR_COMMAND_GET_STATE, &u32_SensorState); // read sensor state

         // print sensor data
         set_cursor(43u, u16_SensorIdx + 7u);
         if (t_SensorData.f32_Temperature != SENSOR_VALUE_INVALID)
         {
            printf("%5.1f", t_SensorData.f32_Temperature);
         }
         else
         {
            printf("---.-");
         }

         set_cursor(52u, u16_SensorIdx + 7u);
         if (t_SensorData.f32_Pressure != SENSOR_VALUE_INVALID)
         {
            printf("%6.1f", t_SensorData.f32_Pressure);
         }
         else
         {
            printf("----.-");
         }

         set_cursor(62u, u16_SensorIdx + 7u);
         if (t_SensorData.f32_Humidity != SENSOR_VALUE_INVALID)
         {
            printf("%5.1f", t_SensorData.f32_Humidity);
         }
         else
         {
            printf("---.-");
         }

         set_cursor(73u, u16_SensorIdx + 7u);
         if (t_SensorData.f32_CO2 != SENSOR_VALUE_INVALID)
         {
            printf("%5.0f", t_SensorData.f32_CO2);
         }
         else
         {
            printf("-----");
         }

         set_cursor(86u, u16_SensorIdx + 7u);
         printf("%08lX", u32_SensorState);

         set_cursor(99u, u16_SensorIdx + 7u);
         if (t_SensorData.u32_TimeStampSec != SENSOR_TIMESTAMP_INVALID)
         {
            const uint32_t u32_AgeSec = get_time_sec() - t_SensorData.u32_TimeStampSec;
            printf("%8lu", u32_AgeSec);
         }
         else
         {
            printf("     ---");
         }
      }

      u8_Key = get_key_nowait();
   }

   return 1;
}

//-----------------------------------------------------------------------------
/*!
   \brief    Sensor setup

   \return   C_NOACT    terminated with no error
*/
//-----------------------------------------------------------------------------
int32_t sensor_setup(void)
{
   const uint16_t u16_SensorCnt = sensor_get_count();

   clr_scr();
   printf("------------------------------------------\r\n");
   printf(" Sensor setup (NVM data)\r\n");
   printf("------------------------------------------\r\n");

   if (u16_SensorCnt == 0u)
   {
      printf(" no sensor found, press any key to terminate...\r\n");
      (void)get_key();
   }
   else
   {
      T_sensor_info t_SensorInfo;
      uint16_t u16_SensorIdx = 0;
      uint8_t u8_Key = 0u;

      printf(" [1] select sensor-\r\n");
      printf(" [2] select sensor+\r\n");
      printf(" [3] edit name\r\n");
      printf("[ESC] terminate setup");

      serial_clear_rx_buf(); // clear RX buffer

      (void)sensor_get_info(u16_SensorIdx, &t_SensorInfo);

      while (u8_Key != 0x1Bu)
      {
         if ((u8_Key == '1') || (u8_Key == '2'))
         {
            if (u8_Key == '1')
            {
               u16_SensorIdx = (u16_SensorIdx > 0u) ? (u16_SensorIdx - 1u) : (u16_SensorCnt -1u);
            }
            else
            {
               u16_SensorIdx++;
               if (u16_SensorIdx >= u16_SensorCnt)
               {
                  u16_SensorIdx = 0u;
               }
            }
            (void)sensor_get_info(u16_SensorIdx, &t_SensorInfo);
         }
         if (u8_Key == '3')
         {
            esp_err_t t_Err;
            char ac_SensorName[SENSOR_NAME_LEN_MAX+1];
            memset(ac_SensorName, ' ', SENSOR_NAME_LEN_MAX);
            ac_SensorName[SENSOR_NAME_LEN_MAX] = 0;
            set_cursor(18u, 9u);
            printf("%s",ac_SensorName);
            set_cursor(18u, 9u);
            t_Err = get_string((uint8_t *)ac_SensorName, SENSOR_NAME_LEN_MAX);
            if (t_Err == ESP_OK)
            {
               (void)sensor_name_set_idx(u16_SensorIdx, ac_SensorName);
               (void)sensor_name_store();
            }
         }

         // print sensor info
         set_cursor(0u, 9u);
         printf("SENSOR%2u: name = %s                       ", u16_SensorIdx+1u, t_SensorInfo.s_Name);
         set_cursor(11u, 10u);
         printf("type = %s, bus = %u             ", t_SensorInfo.pt_Sensor->s_Type, t_SensorInfo.u8_IicBus);

         u8_Key = get_key();
      }
   }

   return 1;
}
