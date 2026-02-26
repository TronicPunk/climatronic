//----------------------------------------------------------------------------------------------------------------------
/*!
   \file    scd4x_interface.h
   \brief   SCD40/SCD41 interface: Glue layer between Sensirion Sensor API and ESP32 driver library
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <string.h>
#include "scd4x_interface.h"
#include "sensirion_i2c.h"
#include "sensirion_common.h"
#include "sensor_name.h"
#include "iic.h"
#include "iic_mux.h"
#include "timer.h"

#include "debug.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define SCD4X_IIC_SCL_TIMEOUT_US    10000u      // IIC SCL timeout in us
#define SCD4X_IIC_TIMEOUT_MS        50u         // IIC transaction timeout in ms

#define SCD4X_STATE_OPERATIONAL     0u          // normal operation, process sensor data
#define SCD4X_STATE_PREPARE_CALIB   1u          // prepare for calibration (enter idle state)
#define SCD4X_STATE_CALIBRATE       2u          // force calibration, expect reference gas with 400ppm CO2
#define SCD4X_STATE_CALIB_RESULT    3u          // read calibration result
#define SCD4X_ENTER_IDLE_TIME_US    500000u     // command time for enter idle state
#define SCD4X_CALIBRATION_TIME_US   400000u     // command time for forced CO2 calibration


/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static esp_err_t m_scd4x_get_mem(T_sensor_instance ** opt_Sensor);
static struct scd4x_dev * m_scd4x_reuse_mem(struct scd4x_dev * const opt_SCD4x_dev);
static void m_scd4x_free_mem(T_sensor_instance * const opt_Sensor);
static esp_err_t m_scd4x_interface_init(const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                        struct scd4x_dev * opt_SCD4x_dev);

// common sensor API
static esp_err_t m_scd4x_sensor_init(const uint8_t ou8_IicBus, const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                     T_sensor_instance ** oppt_Sensor);
static esp_err_t m_scd4x_sensor_process_data(T_sensor_instance * const opt_Sensor);
static esp_err_t m_scd4x_sensor_read_data(T_sensor_instance * const opt_Sensor);
static esp_err_t m_scd4x_sensor_get_data(T_sensor_instance * const opt_Sensor, T_sensor_data * const opt_SensorData);
static esp_err_t m_scd4x_sensor_command(T_sensor_instance * const opt_Sensor, const uint16_t ou16_Command, uint32_t * const  opu32_Parameter);

// low level driver functions
static int16_t m_scd4x_stop_periodic_measurement_nowait(T_sensor_instance * const opt_Sensor);
static int16_t scd4x_perform_forced_recalibration_nowait(T_sensor_instance * const opt_Sensor, uint16_t target_co2_concentration);
static int16_t scd4x_read_recalibration_result(T_sensor_instance * const opt_Sensor, uint16_t* frc_correction);
static int8_t m_scd4x_i2c_read(void *intf_ptr, uint8_t *data, uint32_t length);
static int8_t m_scd4x_i2c_write(void *intf_ptr, const uint8_t *data, uint32_t length);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static const T_sensor_type mt_SCD41_type =
{
   .s_Type = "SCD41",
   .u16_SensorCaps = ((SENSOR_CAP_TEMPERATURE|SENSOR_CAP_HUMIDITY|SENSOR_CAP_CO2) |\
                      (SENSOR_CAP_MODE_SLEEP|SENSOR_CAP_MODE_SINGLE|SENSOR_CAP_MODE_CYCLIC))
};

// delivers common sensor API
static const T_sensor_api mt_SCD4x_api =
{
   .pr_SensorInit = &m_scd4x_sensor_init,
   .pr_SensorProcessData = &m_scd4x_sensor_process_data,
   .pr_SensorGetData = &m_scd4x_sensor_get_data,
   .pr_SensorCommand = &m_scd4x_sensor_command
};

// complete sensor description
const T_sensor_descriptor gt_SCD41_descriptor =
{
   .pt_Sensor = &mt_SCD41_type,
   .pt_SensorApi = &mt_SCD4x_api,
   .u32_UpdateRateMinUs = 5000000u,
   .u32_IicFreqMaxHz = 400000u,
   .u8_IicAddrCount = 1u,
   .au8_IicAddr = { SCD41_I2C_ADDR_62 }
};

/* -- Implementation ------------------------------------------------------------------------------------------------ */

void __force_link_scd4x(void)
{
}

/**
 * select SCD4x sensor device to be accessed by the scd4x_i2s API
 */
esp_err_t scd4x_sensor_select_device(const T_sensor_instance * const opt_Sensor)
{
   esp_err_t t_Err = ESP_FAIL;

   // sensor type valid?
   if (opt_Sensor->pt_SensorDescriptor == &gt_SCD41_descriptor)
   {
      struct scd4x_dev * pt_SCD4x_Dev = opt_Sensor->pv_Handle;

      if (pt_SCD4x_Dev != NULL)
      {
         t_Err = sensirion_i2c_hal_select_device(pt_SCD4x_Dev);
      }
   }

   return t_Err;
}

/**
 * allocate memory for sensor instance data or reuse already allocated memory
 */
static esp_err_t m_scd4x_get_mem(T_sensor_instance ** oppt_Sensor)
{
   esp_err_t t_Err = ESP_OK;
   T_sensor_instance * pt_Sensor = *oppt_Sensor;

   if (pt_Sensor != NULL)
   {
      // check for valid sensor instance data
      if (pt_Sensor->pt_SensorDescriptor == &gt_SCD41_descriptor)
      {
         // try to reuse memory for driver data:
         // return NULL if data not valid - otherwise pointer to empty memory block
         pt_Sensor->pv_Handle = m_scd4x_reuse_mem(pt_Sensor->pv_Handle);
         // free allocated memory for sensor name
         sensor_name_free(pt_Sensor);
      }
      else
      {
         pt_Sensor = NULL;
      }
   }

   if (pt_Sensor == NULL)
   {
      pt_Sensor = calloc(1, sizeof(T_sensor_instance));
      if (pt_Sensor == NULL)
      {
         t_Err = ESP_FAIL;
      }
   }

   if (t_Err == ESP_OK)
   {
      if (pt_Sensor->pv_Handle == NULL)
      {
         void * pv_Handle = calloc(1, sizeof(struct scd4x_dev));
         if (pv_Handle != NULL)
         {
            pt_Sensor->pv_Handle = pv_Handle;
         }
         else
         {
            t_Err = ESP_FAIL;
         }
      }
   }

   if (t_Err == ESP_OK)
   {
      // clear sensor instance data
      sensor_set_data_invalid(&pt_Sensor->t_SensorData);
      pt_Sensor->pt_Next = NULL;
      pt_Sensor->u64_CallTimeUs = 0u;
      pt_Sensor->u32_Parameter = 0u;
      pt_Sensor->u16_SensorState = 0u;
   }
   else
   {
      m_scd4x_free_mem(pt_Sensor);
      pt_Sensor = NULL;
   }

   *oppt_Sensor = pt_Sensor;

   return t_Err;
}

/**
 * free memory and IIC device
 */
static void m_scd4x_free_mem(T_sensor_instance * const opt_Sensor)
{
   if (opt_Sensor != NULL)
   {
      // this will check for valid handle data and also free any IIC device
      (void)m_scd4x_reuse_mem(opt_Sensor->pv_Handle);

      if (opt_Sensor->pv_Handle != NULL)
      {
         free(opt_Sensor->pv_Handle);
         opt_Sensor->pv_Handle = NULL;
      }

      sensor_name_free(opt_Sensor);
      free(opt_Sensor);
   }
}

/**
 * try to reuse memory for sensor handle data
 */
static struct scd4x_dev * m_scd4x_reuse_mem(struct scd4x_dev * const opt_SCD4x_dev)
{
   struct scd4x_dev * pt_SCD4x_dev = opt_SCD4x_dev;

   if (pt_SCD4x_dev != NULL)
   {
      // check for valid handle data
      if ((pt_SCD4x_dev->read == &m_scd4x_i2c_read) &&
          (pt_SCD4x_dev->write == &m_scd4x_i2c_write) &&
          (pt_SCD4x_dev->delay_us == &delay_us))
      {
         if (pt_SCD4x_dev->intf_ptr != NULL)
         {
            i2c_master_bus_rm_device(pt_SCD4x_dev->intf_ptr);
         }

         // reuse memory: clear data
         (void)memset(pt_SCD4x_dev, 0, sizeof(struct scd4x_dev));
      }
      else
      {
         // unknown handle data: can't be used, return NULL
         pt_SCD4x_dev = NULL;
      }
   }

   return pt_SCD4x_dev;
}

/**
 * init the SCD4x interface (IIC)
 */
static esp_err_t m_scd4x_interface_init(const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                        struct scd4x_dev* opt_SCD4x_dev)
{
   esp_err_t t_Err = ESP_FAIL;
   i2c_master_bus_handle_t t_IicBusHandle = iic_get_bus_handle();

   if ((t_IicBusHandle != NULL) && (opt_SCD4x_dev != NULL))
   {
      // create new IIC sensor device
      i2c_device_config_t t_IicDev =
      {
         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
         .device_address = ou8_IicAddr,
         .scl_speed_hz = ou32_IicFreq,
         .scl_wait_us = SCD4X_IIC_SCL_TIMEOUT_US,
         .flags.disable_ack_check = false
      };
      i2c_master_dev_handle_t t_IicDevHandle = NULL;

      t_Err = i2c_master_bus_add_device(t_IicBusHandle, &t_IicDev, &t_IicDevHandle);
      ESP_ERROR_CHECK(t_Err);

      if (t_Err == ESP_OK)
      {
         // register mt_SCD4x_dev
         opt_SCD4x_dev->i2c_address = ou8_IicAddr;
         opt_SCD4x_dev->intf_ptr = t_IicDevHandle;
         opt_SCD4x_dev->read = &m_scd4x_i2c_read;
         opt_SCD4x_dev->write = &m_scd4x_i2c_write;
         opt_SCD4x_dev->delay_us = &delay_us;
      }
   }

   return t_Err;
}

/*!
 * sensor API: init specific SCD4x device
 */
static esp_err_t m_scd4x_sensor_init(const uint8_t ou8_IicBus, const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                     T_sensor_instance ** oppt_Sensor)
{
   T_sensor_instance * pt_Sensor = *oppt_Sensor;
   esp_err_t t_Err = m_scd4x_get_mem(&pt_Sensor);

   if (t_Err == ESP_OK)
   {
      t_Err = iic_mux_set(ou8_IicBus);
      if (t_Err == ESP_OK)
      {
         struct scd4x_dev * pt_SCD4x_dev = pt_Sensor->pv_Handle;
         pt_SCD4x_dev->i2c_bus = ou8_IicBus;
         t_Err = ESP_FAIL;

         if (m_scd4x_interface_init(ou8_IicAddr, ou32_IicFreq, pt_SCD4x_dev) == ESP_OK)
         {
            // select SCD4x device for driver API
            if (sensirion_i2c_hal_select_device(pt_SCD4x_dev) == 0)
            {
               // force sensor into idle state (note: this function call waits 500ms!)
               if (scd4x_stop_periodic_measurement() == 0)
               {
                  // read device information
                  if (scd4x_get_sensor_variant_raw(&pt_SCD4x_dev->chip_id) == 0)
                  {
                     if (scd4x_get_serial_number(pt_SCD4x_dev->chip_serial_num, 3u) == 0u)
                     {
                        // start operation mode
                        if (scd4x_start_periodic_measurement() == 0)
                        {
                           // setup sensor instance data
                           pt_Sensor->pt_SensorDescriptor = &gt_SCD41_descriptor;
                           pt_Sensor->u32_IicFreq = ou32_IicFreq;
                           pt_Sensor->u8_IicBus = ou8_IicBus;
                           pt_Sensor->u8_IicAddr = ou8_IicAddr;
                           pt_Sensor->s_SensorName = sensor_name_create(pt_Sensor);
                           // setup process timer
                           pt_Sensor->u64_CallTimeUs = esp_timer_get_time() + gt_SCD41_descriptor.u32_UpdateRateMinUs;
                           pt_Sensor->u16_SensorState = SCD4X_STATE_OPERATIONAL;
                           t_Err = ESP_OK;
                        }
                     }
                  }
               }
            }
         }
      }

      if (t_Err != ESP_OK)
      {
         m_scd4x_free_mem(pt_Sensor); // on error: free all allocated resources
      }
   }

   if (t_Err != ESP_OK)
   {
      pt_Sensor = NULL;
   }

   *oppt_Sensor = pt_Sensor;

   return t_Err;
}

/*!
 * SCD4x process data task
 */
static esp_err_t m_scd4x_sensor_process_data(T_sensor_instance * const opt_Sensor)
{
   esp_err_t t_Err = ESP_ERR_NOT_FINISHED;
   // get system time
   const uint64_t u64_SysTime = esp_timer_get_time();

   if (u64_SysTime >= opt_Sensor->u64_CallTimeUs)
   {
      // if time elapsed -> check sensor state machine
      // register SCD4x device handle for driver API and set I2C mux
      if (sensirion_i2c_hal_select_device(opt_Sensor->pv_Handle) == ESP_OK)
      {
         if (opt_Sensor->u16_SensorState == SCD4X_STATE_OPERATIONAL)
         {
            // try to read sensor data
            t_Err = m_scd4x_sensor_read_data(opt_Sensor);
         }
         else if (opt_Sensor->u16_SensorState == SCD4X_STATE_PREPARE_CALIB)
         {
            // prepare for calibration, enter idle state
            if (m_scd4x_stop_periodic_measurement_nowait(opt_Sensor) == 0)
            {
               opt_Sensor->u64_CallTimeUs = u64_SysTime + SCD4X_ENTER_IDLE_TIME_US;
               opt_Sensor->u16_SensorState = SCD4X_STATE_CALIBRATE;
               t_Err = ESP_OK;
            }
            else
            {
               t_Err = ESP_FAIL;
            }
         }
         else if (opt_Sensor->u16_SensorState == SCD4X_STATE_CALIBRATE)
         {
            // force calibration
            if (scd4x_perform_forced_recalibration_nowait(opt_Sensor, (uint16_t)opt_Sensor->u32_Parameter) == 0)
            {
               opt_Sensor->u64_CallTimeUs = u64_SysTime + SCD4X_CALIBRATION_TIME_US;
               opt_Sensor->u16_SensorState = SCD4X_STATE_CALIB_RESULT;
               t_Err = ESP_OK;
            }
            else
            {
               t_Err = ESP_FAIL;
            }
         }
         else if (opt_Sensor->u16_SensorState == SCD4X_STATE_CALIB_RESULT)
         {
            uint16_t u16_FrcCorr;
            // force calibration
            if (scd4x_read_recalibration_result(opt_Sensor, &u16_FrcCorr) == 0)
            {
               scd4x_start_periodic_measurement();
               opt_Sensor->u64_CallTimeUs = u64_SysTime + gt_SCD41_descriptor.u32_UpdateRateMinUs;
               opt_Sensor->u16_SensorState = SCD4X_STATE_OPERATIONAL;
               opt_Sensor->u32_Parameter = (uint16_t)(u16_FrcCorr + 0x8000u);
               t_Err = ESP_OK;
            }
            else
            {
               t_Err = ESP_FAIL;
            }
         }
      }
      else
      {
         t_Err = ESP_FAIL;
      }
   }

   return t_Err;
}

/*!
 * read sensor data from SCD4x registers
 */
static esp_err_t m_scd4x_sensor_read_data(T_sensor_instance * const opt_Sensor)
{
   esp_err_t t_Err = ESP_FAIL;
   bool q_DataReady = false;

   if (scd4x_get_data_ready_status(&q_DataReady) == 0)
   {
      if (q_DataReady == true)
      {
         uint16_t u16_CO2;
         int32_t s32_Temp;
         int32_t s32_Hum;

         if (scd4x_read_measurement(&u16_CO2, &s32_Temp, &s32_Hum) == 0)
         {
            T_sensor_data * const pt_SensorData = &opt_Sensor->t_SensorData;

            pt_SensorData->f32_Temperature = (float)s32_Temp * 0.001f;
            pt_SensorData->f32_Pressure = SENSOR_VALUE_INVALID;
            pt_SensorData->f32_Humidity = (float)s32_Hum * 0.001f;
            pt_SensorData->f32_CO2 = (float)u16_CO2;
            pt_SensorData->f32_IAQ = SENSOR_VALUE_INVALID;
            pt_SensorData->u32_TimeStampSec = get_time_sec();

            opt_Sensor->u64_CallTimeUs = esp_timer_get_time() + opt_Sensor->pt_SensorDescriptor->u32_UpdateRateMinUs;
            t_Err = ESP_OK;
         }
      }
      else
      {
         t_Err = ESP_ERR_NOT_FINISHED;
      }
   }

   return t_Err;
}

/*!
 * get all SCD4x sensor data
 */
static esp_err_t m_scd4x_sensor_get_data(T_sensor_instance * const opt_Sensor, T_sensor_data * const opt_SensorData)
{
   *opt_SensorData = opt_Sensor->t_SensorData;
   // at least one data sample available? -> return ESP_OK
   return (opt_Sensor->t_SensorData.u32_TimeStampSec == SENSOR_TIMESTAMP_INVALID) ? ESP_ERR_NOT_FINISHED : ESP_OK;
}

/*!
 * SCD4x command interface
 */
static esp_err_t m_scd4x_sensor_command(T_sensor_instance * const opt_Sensor, const uint16_t ou16_Command, uint32_t * const  opu32_Parameter)
{
   esp_err_t t_Err = ESP_ERR_NOT_SUPPORTED;

   if (ou16_Command == SENSOR_COMMAND_GET_STATE)
   {
      *opu32_Parameter = ((uint32_t)opt_Sensor->u16_SensorState << 16u) | (opt_Sensor->u32_Parameter & 0x0000FFFFuL);
      t_Err = ESP_OK;
   }
   else if (ou16_Command == SENSOR_COMMAND_CALIBRATE_CO2)
   {
      if (opt_Sensor->u16_SensorState == SCD4X_STATE_OPERATIONAL)
      {
         opt_Sensor->u32_Parameter = *opu32_Parameter;
         opt_Sensor->u16_SensorState = SCD4X_STATE_PREPARE_CALIB;
         opt_Sensor->u64_CallTimeUs = 0uLL;
         t_Err = ESP_OK;
      }
      else
      {
         t_Err = ESP_ERR_NOT_FINISHED;
      }
   }

   return t_Err;
}

/*!
 * I2C read function map to ESP32 platform
 */
static int8_t m_scd4x_i2c_read(void *intf_ptr, uint8_t *data, uint32_t length)
{
   int8_t t_Result = ESP_OK;

   if (i2c_master_receive(intf_ptr, data, length, SCD4X_IIC_TIMEOUT_MS) != ESP_OK)
   {
      t_Result = ESP_FAIL;
   }

   return t_Result;
}

/*!
 * I2C write function map to ESP32 platform
 */
static int8_t m_scd4x_i2c_write(void *intf_ptr, const uint8_t *data, uint32_t length)
{
   int8_t t_Result = ESP_OK;

   if (i2c_master_transmit(intf_ptr, data, length, SCD4X_IIC_TIMEOUT_MS) != ESP_OK)
   {
      t_Result = ESP_FAIL;
   }

   return t_Result;
}

/*!
 * SCD4x send "stop periodic measurement" command but do not wait for command to finish (500ms!)
 */
static int16_t m_scd4x_stop_periodic_measurement_nowait(T_sensor_instance * const opt_Sensor)
{
   int16_t local_error;
   uint8_t buffer[9];
   uint16_t local_offset = 0;
   local_offset = sensirion_i2c_add_command16_to_buffer(buffer, local_offset, 0x3f86);
   local_error = sensirion_i2c_write_data(opt_Sensor->u8_IicAddr, buffer, local_offset);

   return local_error;
}

/*!
 * SCD4x send "perform forced recalibration" command but do not wait for command to finish (400ms!)
 */
static int16_t scd4x_perform_forced_recalibration_nowait(T_sensor_instance * const opt_Sensor, uint16_t target_co2_concentration)
{
   int16_t local_error;
   uint8_t buffer[9];
   uint16_t local_offset = 0;
   local_offset = sensirion_i2c_add_command16_to_buffer(buffer, local_offset, 0x362f);
   local_offset = sensirion_i2c_add_uint16_t_to_buffer(buffer, local_offset, target_co2_concentration);
   local_error = sensirion_i2c_write_data(opt_Sensor->u8_IicAddr, buffer, local_offset);
   return local_error;
}

/*!
 * SCD4x read recalibration result
 */
static int16_t scd4x_read_recalibration_result(T_sensor_instance * const opt_Sensor, uint16_t* frc_correction)
{
   uint8_t buffer[9];
   int16_t local_error = sensirion_i2c_read_data_inplace(opt_Sensor->u8_IicAddr, buffer, 2);
   if (local_error != 0) {
       return local_error;
   }
   *frc_correction = sensirion_common_bytes_to_uint16_t(&buffer[0]);
   return local_error;
}
