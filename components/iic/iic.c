//----------------------------------------------------------------------------------------------------------------------
/*!
   \file
   \brief provide functions for multiplexed IIC communication
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "iic.h"
#include "driver/gpio.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define IIC_PORT                 I2C_NUM_0      // ESP32 IIC port
#define IIC_SDA_GPIO             GPIO_NUM_21    // IIC SDA GPIO
#define IIC_SCL_GPIO             GPIO_NUM_22    // IIC SDA GPIO

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */
static i2c_master_bus_handle_t mt_IicBusHandle = NULL;


/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * configure the ESP32 IIC master interface
 */
esp_err_t iic_init(void)
{
   esp_err_t t_Err;
   static const i2c_master_bus_config_t i2c_mst_config =
   {
      .i2c_port = IIC_PORT,
      .sda_io_num = IIC_SDA_GPIO,
      .scl_io_num = IIC_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7u,
      .intr_priority = 0,
      .flags.enable_internal_pullup = true,
      .flags.allow_pd = false      
   };

   // init IIC master bus
   t_Err = i2c_new_master_bus(&i2c_mst_config, &mt_IicBusHandle);
   ESP_ERROR_CHECK(t_Err);

   return t_Err;
}

/**
 * return the IIC master bus handle
 * return NULL if IIC master is not initialized
 */
i2c_master_bus_handle_t iic_get_bus_handle(void)
{
   return mt_IicBusHandle;
}
