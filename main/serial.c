//----------------------------------------------------------------------------------------------------------------------
/*!
   \file
   \brief provide serial communication for terminal connection
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <stdio.h>
#include <fcntl.h>
#include "serial.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * configure the stdio UART communication
 */
void serial_init(void)
{
   const int stdin_fileno = fileno(stdin);

   /* Enable blocking mode on stdin and stdout */
   fcntl(fileno(stdout), F_SETFL, 0);
   fcntl(stdin_fileno, F_SETFL, 0);

   // Set stdin to unbuffered
   setvbuf(stdin, NULL, _IONBF, 0);

   // Flush input
   assert(serial_clear_rx_buf() == 0);

   // Flush output
   assert(serial_flush_tx_buf() == 0);
}

int serial_flush_tx_buf(void)
{
   return fflush(stdout);
}

int serial_clear_rx_buf(void)
{
   return fflush(stdin);
}
