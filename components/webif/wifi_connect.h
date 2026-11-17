//----------------------------------------------------------------------------------------------------------------------
/*!
   \file  wifi_connect.h
   \brief provide functions for WIFI connection
*/
//----------------------------------------------------------------------------------------------------------------------

#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "wifi_data.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

#define WIFI_STATUS_LED_CONNECT     0x0028F028uL   // WIFI connected: LED light green
#define WIFI_STATUS_LED_MQTT        0x00006060uL   // WIFI connected + MQTT connected: LED blue

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

void wifi_connect(T_wifi_data * const opt_DeviceData);
void wifi_reset(void);
void start_mdns_service(const char* const oac_HostName);
void stop_mdns_service(void);

/* -- Implementation ------------------------------------------------------------------------------------------------ */

#endif /* WIFI_CONNECT_H */
