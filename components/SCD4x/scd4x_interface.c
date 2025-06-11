//----------------------------------------------------------------------------------------------------------------------
/*!
   \file    scd4x_interface.h
   \brief   SCD40/SCD41 interface: Glue layer between Sensirion Sensor API and ESP32 driver library
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <string.h>
#include "scd4x_interface.h"
#include "iic.h"
#include "iic_mux.h"
#include "timer.h"

#include "debug.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define SCD4X_IIC_SCL_TIMEOUT_US   (50000u)    // IIC SCL timeout in us
#define SCD4X_IIC_TIMEOUT_MS       (50u)       // IIC transaction timeout in ms


/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static esp_err_t m_scd4x_interface_init(const uint16_t ou16_IicAddr, const uint32_t ou32_IicSclFreq, struct scd4x_dev * opt_SCD4x_dev);
static esp_err_t m_scd4x_get_mem(T_sensor_instance * const opt_SensorInstance);
static void m_scd4x_free_mem(T_sensor_instance * const opt_SensorInstance);
static struct scd4x_dev * m_scd4x_reuse_mem(struct scd4x_dev * const opt_SCD4x_dev);

// common sensor API
static esp_err_t m_scd4x_sensor_init(const T_sensor_def * const opt_SensorInfo);
static esp_err_t m_scd4x_sensor_process_data(const T_sensor_def * const opt_SensorInfo);
static esp_err_t m_scd4x_sensor_get_data(const T_sensor_def * const opt_SensorInfo, T_sensor_data * const opt_SensorData);

// low level driver functions
static int8_t m_scd4x_i2c_read(void *intf_ptr, uint8_t *data, uint32_t length);
static int8_t m_scd4x_i2c_write(void *intf_ptr, const uint8_t *data, uint32_t length);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

const T_sensor_type gt_SCD41_type = 
{
   .s_Type = "SCD41",
   .u16_SensorCaps = ((SENSOR_CAP_TEMPERATURE|SENSOR_CAP_HUMIDITY|SENSOR_CAP_CO2) |\
                      (SENSOR_CAP_MODE_SLEEP|SENSOR_CAP_MODE_SINGLE|SENSOR_CAP_MODE_CYCLIC))
};

// delivers common sensor API
const T_sensor_api gt_SCD4x_api =
{
   .pr_SensorInit = &m_scd4x_sensor_init,
   .pr_SensorProcessData = &m_scd4x_sensor_process_data,
   .pr_SensorGetData = &m_scd4x_sensor_get_data
};


/* -- Implementation ------------------------------------------------------------------------------------------------ */

void __force_link_scd4x(void)
{
}

/**
 * select SCD4x sensor device to be accessed by the scd4x_i2s API
 */
esp_err_t scd4x_sensor_select_device(const T_sensor_def * const opt_SensorDef)
{
   esp_err_t t_Err = ESP_FAIL;   
   
   // sensor type valid?
   if (opt_SensorDef->t_SensorInfo.pt_Sensor == &gt_SCD41_type)
   {
      struct scd4x_dev * pt_SCD4x_Dev = opt_SensorDef->pt_SensorInstance->pv_Handle;

      if (pt_SCD4x_Dev != NULL)
      {
         t_Err = sensirion_i2c_hal_select_device(pt_SCD4x_Dev);
      }
   }

   return t_Err;
}


/**
 * init the SCD4x interface (IIC)
 */
static esp_err_t m_scd4x_interface_init(const uint16_t ou16_IicAddr, const uint32_t ou32_IicSclFreq, struct scd4x_dev* opt_SCD4x_dev)
{
   esp_err_t t_Err = ESP_FAIL;
   i2c_master_bus_handle_t t_IicBusHandle = iic_get_bus_handle();

   if ((t_IicBusHandle != NULL) && (opt_SCD4x_dev != NULL))
   {
      // create new IIC sensor device
      i2c_device_config_t t_IicDev =
      {
         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
         .device_address = ou16_IicAddr,
         .scl_speed_hz = ou32_IicSclFreq,
         .scl_wait_us = SCD4X_IIC_SCL_TIMEOUT_US,
         .flags.disable_ack_check = false
      };
      i2c_master_dev_handle_t t_IicDevHandle = NULL;
   
      t_Err = i2c_master_bus_add_device(t_IicBusHandle, &t_IicDev, &t_IicDevHandle);
      ESP_ERROR_CHECK(t_Err);
   
      if (t_Err == ESP_OK)
      {
         // register mt_SCD4x_dev
         opt_SCD4x_dev->i2c_address = (uint8_t)ou16_IicAddr;
         opt_SCD4x_dev->intf_ptr = t_IicDevHandle;
         opt_SCD4x_dev->read = &m_scd4x_i2c_read;
         opt_SCD4x_dev->write = &m_scd4x_i2c_write;
         opt_SCD4x_dev->delay_us = &delay_us;
      }
   }

   return t_Err;
}

/**
 * allocate memory for sensor instance data or reuse already allocated memory
 */
static esp_err_t m_scd4x_get_mem(T_sensor_instance * const opt_SensorInstance)
{
   esp_err_t t_Err = ESP_OK;
   struct scd4x_dev * pt_SCD4x_dev = m_scd4x_reuse_mem(opt_SensorInstance->pv_Handle);

   if (pt_SCD4x_dev == NULL)
   {
      pt_SCD4x_dev = calloc(1, sizeof(struct scd4x_dev));
      if (pt_SCD4x_dev == NULL)
      {
         t_Err = ESP_FAIL;
      }
      opt_SensorInstance->pv_Handle = pt_SCD4x_dev;
   }

   // clear sensor instance data
   sensor_set_data_invalid(&opt_SensorInstance->t_SensorData);
   opt_SensorInstance->u64_TimeStampUs = 0u;
   opt_SensorInstance->u32_NextCallUs = 0u;

   return t_Err;
}

/**
 * free memory and IIC device
 */
static void m_scd4x_free_mem(T_sensor_instance * const opt_SensorInstance)
{
   // this will check for valid handle data and also free any IIC device
   struct scd4x_dev * pt_SCD4x_dev = m_scd4x_reuse_mem(opt_SensorInstance->pv_Handle);

   if (pt_SCD4x_dev != NULL)
   {
      free(pt_SCD4x_dev);
      opt_SensorInstance->pv_Handle = NULL;
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

/*!
 * sensor API: init specific SCD4x device
 */
static esp_err_t m_scd4x_sensor_init(const T_sensor_def * const opt_SensorDef)
{
   const T_sensor_info * const pt_SensorInfo = &opt_SensorDef->t_SensorInfo;
   T_sensor_instance * const pt_SensorInstance = opt_SensorDef->pt_SensorInstance;   
   esp_err_t t_Err = m_scd4x_get_mem(pt_SensorInstance);

   if (t_Err == ESP_OK)
   {
      t_Err = iic_mux_set(pt_SensorInfo->u8_IicBus);
      if (t_Err == ESP_OK)
      {
         struct scd4x_dev * pt_SCD4x_dev = pt_SensorInstance->pv_Handle;
         pt_SCD4x_dev->i2c_bus = pt_SensorInfo->u8_IicBus;
         t_Err = ESP_FAIL;

         if (m_scd4x_interface_init(pt_SensorInfo->u8_IicAddr, pt_SensorInfo->u32_IicSclFreq, pt_SCD4x_dev) == ESP_OK)
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
                           // on success: setup process timer
                           pt_SensorInstance->u64_TimeStampUs = esp_timer_get_time();
                           pt_SensorInstance->u32_NextCallUs = opt_SensorDef->u32_UpdateRateUs;
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
         m_scd4x_free_mem(pt_SensorInstance); // on error: free all allocated resources
      }
   }

   return t_Err;
}

/*!
 * SCD4x process data task
 */
static esp_err_t m_scd4x_sensor_process_data(const T_sensor_def * const opt_SensorDef)
{
   esp_err_t t_Err = ESP_ERR_NOT_FINISHED;
   T_sensor_instance * const pt_SensorInstance = opt_SensorDef->pt_SensorInstance;
   // get system time
   const uint64_t u64_SysTime = esp_timer_get_time();

   if (u64_SysTime >= pt_SensorInstance->u64_TimeStampUs + pt_SensorInstance->u32_NextCallUs)
   {
      // if time elapsed -> get new sensor data
      struct scd4x_dev * pt_SCD4x_dev = pt_SensorInstance->pv_Handle;
      T_sensor_data * const pt_SensorData = &pt_SensorInstance->t_SensorData;
      t_Err = ESP_FAIL;
      // register SCD4x device handle for driver API and set I2C mux
      if (sensirion_i2c_hal_select_device(pt_SCD4x_dev) == ESP_OK)
      {
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
                  pt_SensorData->f32_Temperature = (float)s32_Temp * 0.001f;
                  pt_SensorData->f32_Pressure = SENSOR_VALUE_INVALID;
                  pt_SensorData->f32_Humidity = (float)s32_Hum * 0.001f;
                  pt_SensorData->f32_CO2 = (float)u16_CO2;
                  pt_SensorData->f32_IAQ = SENSOR_VALUE_INVALID;
                  pt_SensorData->u32_AgeSec = 0u;

                  pt_SensorInstance->u64_TimeStampUs = u64_SysTime;
                  pt_SensorInstance->u32_NextCallUs = opt_SensorDef->u32_UpdateRateUs;
                  t_Err = ESP_OK;
               }
            }
            else
            {
               t_Err = ESP_ERR_NOT_FINISHED;
            }
         }
      }

      if (t_Err == ESP_FAIL)
      {
         sensor_set_data_invalid(pt_SensorData);
         pt_SensorInstance->u32_NextCallUs += (opt_SensorDef->u32_UpdateRateUs / 2u);
      }
   }

   return t_Err;
}

/*!
 * get all SCD4x sensor data
 */
static esp_err_t m_scd4x_sensor_get_data(const T_sensor_def * const opt_SensorDef, T_sensor_data * const opt_SensorData)
{
   esp_err_t t_Err = ESP_ERR_NOT_FINISHED;
   T_sensor_instance * const pt_SensorInstance = opt_SensorDef->pt_SensorInstance;

   // at least one data sample available?
   if (pt_SensorInstance->t_SensorData.u32_AgeSec != SENSOR_TIMESTAMP_INVALID)
   {
      // calculate data sample age
      const uint32_t u32_AgeSec = (uint32_t)((esp_timer_get_time() - pt_SensorInstance->u64_TimeStampUs)/1000000uL);
      pt_SensorInstance->t_SensorData.u32_AgeSec = u32_AgeSec;
      t_Err = ESP_OK;
   }

   *opt_SensorData = pt_SensorInstance->t_SensorData;

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
