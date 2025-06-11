/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SENSIRION_I2C_HAL_H
#define SENSIRION_I2C_HAL_H

#include "sensirion_config.h"
#include "esp_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/*! @name           Defines                                                   */
/******************************************************************************/
#define USE_FAST_CRC8   (1)


/******************************************************************************/
/*! @name           Types, Function Pointers                                  */
/******************************************************************************/

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in, out] intf_ptr   : Void pointer for interface device handle
 * @param[out] reg_data       : Pointer to data buffer where read data is stored.
 * @param[in] len             : Number of bytes of data to be read.
 *
 * @retval   0 -> Success.
 * @retval Non zero value -> Fail.
 */
typedef int8_t (*scd4x_read_fptr_t)(void *intf_ptr, uint8_t *reg_data, uint32_t len);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in, out] intf_ptr   : Void pointer for interface device handle
 * @param[in] reg_data        : Pointer to data buffer in which data to be written is stored.
 * @param[in] len             : Number of bytes of data to be written.
 *
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 */
typedef int8_t (*scd4x_write_fptr_t)(void *intf_ptr, const uint8_t *reg_data, uint32_t len);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period          : Delay in microseconds.
 */
typedef void (*scd4x_delay_us_fptr_t)(uint32_t period);


/*!
 * @brief scd4x device structure
 */
typedef struct scd4x_dev
{
   /*! I2C bus number*/
   uint8_t i2c_bus;
   /*! Device Address */
   uint8_t i2c_address;
   /*! Chip Id */
   uint16_t chip_id;
   /*! Chip serial number */
   uint16_t chip_serial_num[3];

   /*! I2C device handle */
   void *intf_ptr;

   /*! Read function pointer */
   scd4x_read_fptr_t read;

   /*! Write function pointer */
   scd4x_write_fptr_t write;

   /*! Delay function pointer */
   scd4x_delay_us_fptr_t delay_us;
} T_scd4x_dev;


/******************************************************************************/
/*! @name           Function Prototypes                                       */
/******************************************************************************/

#ifdef USE_FAST_CRC8
/**
 * Substitute Sensitrion standard CRC by a fast table driven CRC
 */
uint8_t sensirion_i2c_generate_crc(const uint8_t* data, uint16_t count);
#endif

/**
 * Select the current iic sensor device by opt_SCD4x_Dev
 * All following i2c operations will be directed to this device
 *
 * @param opt_SCD4x_Dev SCD4x device handle
 * @returns             0 on success, an error code otherwise
 */
esp_err_t sensirion_i2c_hal_select_device(const T_scd4x_dev * const opt_SCD4x_Dev);

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_hal_init(void);

/**
 * Release all resources initialized by sensirion_i2c_hal_init().
 */
void sensirion_i2c_hal_free(void);

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint16_t count);

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint16_t count);

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution approximately, but no less than, the given time.
 *
 * When using hardware i2c:
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * When using software i2c:
 * The precision needed depends on the desired i2c frequency, i.e. should be
 * exact to about half a clock cycle (defined in
 * `SENSIRION_I2C_CLOCK_PERIOD_USEC` in `sensirion_sw_i2c_gpio.h`).
 *
 * Example with 400kHz requires a precision of 1 / (2 * 400kHz) == 1.25usec.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_i2c_hal_sleep_usec(uint32_t useconds);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SENSIRION_I2C_HAL_H */
