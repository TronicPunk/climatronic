//----------------------------------------------------------------------------------------------------------------------
/*!
   \file
   \brief provide functions for IIC multiplexer control
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include "iic_mux.h"
#include "iic.h"
#include "driver/gpio.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define IIC_MUX_RST_GPIO         GPIO_NUM_14    // IIC MUX reset GPIO
#define IIC_MUX_ADDR             (0x70u)        // IIC multiplexer device address
#define IIC_MUX_SCL_FREQ         (400000u)      // IIC multiplexer SCL frequency
#define IIC_MUX_SCL_TIMEOUT_US   (10000u)       // IIC multiplexer SCL timeout in us
#define IIC_MUX_TIMEOUT_MS       (50u)          // IIC multiplexer transaction timeout in ms

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */
static i2c_master_dev_handle_t mt_IicDevHandle_Mux = NULL;
static uint8_t mu8_MuxChannel = IIC_MUX_CHANNEL_INVALID;

/* -- Implementation ------------------------------------------------------------------------------------------------ */

/**
 * configure the ESP32 IIC master interface
 */
esp_err_t iic_mux_init(void)
{
   static const i2c_device_config_t t_IicDev_Mux =
   {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = IIC_MUX_ADDR,
      .scl_speed_hz = IIC_MUX_SCL_FREQ,
      .scl_wait_us = IIC_MUX_SCL_TIMEOUT_US,
      .flags.disable_ack_check = false
   };
   esp_err_t t_Err;
   const i2c_master_bus_handle_t t_IicBusHandle = iic_get_bus_handle();

   if (t_IicBusHandle == NULL)
   {
      t_Err = ESP_FAIL;
   }
   else
   {
      // init IIC multiplexer GPIO for reset signal
      gpio_reset_pin(IIC_MUX_RST_GPIO);
      gpio_set_level(IIC_MUX_RST_GPIO, 1u);
      /* Set the GPIO as a push/pull output */
      gpio_set_direction(IIC_MUX_RST_GPIO, GPIO_MODE_OUTPUT);

      // reset the IIC multiplexer
      iic_mux_reset();

      // init IIC multiplexer device
      t_Err = i2c_master_bus_add_device(t_IicBusHandle, &t_IicDev_Mux, &mt_IicDevHandle_Mux);
      ESP_ERROR_CHECK(t_Err);
   }

   return t_Err;
}

/**
 * reset the IIC multiplexer
 */
void iic_mux_reset(void)
{
   gpio_set_level(IIC_MUX_RST_GPIO, 0u);
   // set the /RST pin low for at least 6ns (at least 2 cycles at 240MHz CPU clock)
   asm volatile("nop");
   asm volatile("nop");
   asm volatile("nop");
   gpio_set_level(IIC_MUX_RST_GPIO, 1u);

   mu8_MuxChannel = IIC_MUX_CHANNEL_INVALID;
}

/**
 * set the IIC multiplexer state
 */
esp_err_t iic_mux_set(const uint8_t ou8_Channel)
{
   esp_err_t t_Err = ESP_OK;

   if (ou8_Channel != mu8_MuxChannel)
   {
      const uint8_t u8_MuxData = (ou8_Channel < IIC_MUX_CHANNEL_MAX) ? (uint8_t)(1u << ou8_Channel) : 0u;
      t_Err = i2c_master_transmit(mt_IicDevHandle_Mux, &u8_MuxData, 1, IIC_MUX_TIMEOUT_MS);

      if (t_Err == ESP_OK)
      {
         mu8_MuxChannel = ou8_Channel;
      }
   }

   return t_Err;
}

/**
 * read the IIC multiplexer state
 */
esp_err_t iic_mux_get(uint8_t * const opu8_Channel)
{
   uint8_t u8_MuxData;
   uint8_t u8_Channel = IIC_MUX_CHANNEL_INVALID;
   esp_err_t t_Err;

   t_Err = i2c_master_receive(mt_IicDevHandle_Mux, &u8_MuxData, 1, IIC_MUX_TIMEOUT_MS);

   if (t_Err == ESP_OK)
   {
      for (u8_Channel = 0u; u8_Channel < IIC_MUX_CHANNEL_MAX; u8_Channel++)
      {
         if (u8_MuxData == (1u << u8_Channel))
         {
            break;
         }
      }
   }

   *opu8_Channel = (u8_Channel < IIC_MUX_CHANNEL_MAX) ? u8_Channel : IIC_MUX_CHANNEL_INVALID;

   return t_Err;
}
