//----------------------------------------------------------------------------------------------------------------------
/*!
   \file    bme280_interface.c
   \brief   BME280 interface: Glue layer between BOSCH Sensor API and ESP32 driver library
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <string.h>
#include "bme280_interface.h"
#include "iic.h"
#include "iic_mux.h"
#include "timer.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define BME280_IIC_SCL_TIMEOUT_US   (50000u)    // IIC SCL timeout in us
#define BME280_IIC_TIMEOUT_MS       (50u)       // IIC transaction timeout in ms

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static esp_err_t m_bme280_interface_init(const uint16_t ou16_IicAddr, const uint32_t ou32_IicSclFreq, struct bme280_dev * opt_BME280_dev);
static esp_err_t m_bme280_get_mem(T_sensor_instance * const opt_SensorInstance);
static void m_bme280_free_mem(T_sensor_instance * const opt_SensorInstance);
static struct bme280_dev * m_bme280_reuse_mem(struct bme280_dev * const opt_BME280_dev);

// common sensor API
static esp_err_t m_bme280_sensor_init(const T_sensor_def * const opt_SensorInfo);
static esp_err_t m_bme280_sensor_process_data(const T_sensor_def * const opt_SensorInfo);
static esp_err_t m_bme280_sensor_get_data(const T_sensor_def * const opt_SensorInfo, T_sensor_data * const opt_SensorData);

// low level driver functions
static BME280_INTF_RET_TYPE m_bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
static BME280_INTF_RET_TYPE m_bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
static void m_bme280_delay_us(uint32_t u32_delay_us, void *intf_ptr);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

const T_sensor_type gt_BME280_type = 
{
   .s_Type = "BME280",
   .u16_SensorCaps = ((SENSOR_CAP_TEMPERATURE|SENSOR_CAP_PRESSURE|SENSOR_CAP_HUMIDITY) |\
                      (SENSOR_CAP_MODE_SLEEP|SENSOR_CAP_MODE_SINGLE|SENSOR_CAP_MODE_CYCLIC))
};

// delivers common sensor API
const T_sensor_api gt_BME280_api =
{
   .pr_SensorInit = &m_bme280_sensor_init,
   .pr_SensorProcessData = &m_bme280_sensor_process_data,
   .pr_SensorGetData = &m_bme280_sensor_get_data
};


/* -- Implementation ------------------------------------------------------------------------------------------------ */

void __force_link_bme280(void)
{
}

/**
 * init the BME280 interface (IIC)
 */
static esp_err_t m_bme280_interface_init(const uint16_t ou16_IicAddr, const uint32_t ou32_IicSclFreq, struct bme280_dev* opt_BME280_dev)
{
   esp_err_t t_Err = ESP_FAIL;
   i2c_master_bus_handle_t t_IicBusHandle = iic_get_bus_handle();

   if ((t_IicBusHandle != NULL) && (opt_BME280_dev != NULL))
   {
      // create new IIC sensor device
      i2c_device_config_t t_IicDev =
      {
         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
         .device_address = ou16_IicAddr,
         .scl_speed_hz = ou32_IicSclFreq,
         .scl_wait_us = BME280_IIC_SCL_TIMEOUT_US,
         .flags.disable_ack_check = false
      };
      i2c_master_dev_handle_t t_IicDevHandle = NULL;
   
      t_Err = i2c_master_bus_add_device(t_IicBusHandle, &t_IicDev, &t_IicDevHandle);
      ESP_ERROR_CHECK(t_Err);
   
      if (t_Err == ESP_OK)
      {
         // register mt_BME280_dev
         opt_BME280_dev->intf = BME280_I2C_INTF;
         opt_BME280_dev->intf_ptr = t_IicDevHandle;
         opt_BME280_dev->read = &m_bme280_i2c_read;
         opt_BME280_dev->write = &m_bme280_i2c_write;
         opt_BME280_dev->delay_us = &m_bme280_delay_us;
      }
   }

   return t_Err;
}

/**
 * allocate memory for sensor instance data or reuse already allocated memory
 */
static esp_err_t m_bme280_get_mem(T_sensor_instance * const opt_SensorInstance)
{
   esp_err_t t_Err = ESP_OK;
   struct bme280_dev * pt_BME280_dev = m_bme280_reuse_mem(opt_SensorInstance->pv_Handle);

   if (pt_BME280_dev == NULL)
   {
      pt_BME280_dev = calloc(1, sizeof(struct bme280_dev));
      if (pt_BME280_dev == NULL)
      {
         t_Err = ESP_FAIL;
      }
      opt_SensorInstance->pv_Handle = pt_BME280_dev;
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
static void m_bme280_free_mem(T_sensor_instance * const opt_SensorInstance)
{
   // this will check for valid handle data and also free any IIC device
   struct bme280_dev * pt_BME280_dev = m_bme280_reuse_mem(opt_SensorInstance->pv_Handle);

   if (pt_BME280_dev != NULL)
   {
      free(pt_BME280_dev);
      opt_SensorInstance->pv_Handle = NULL;
   }
}

/**
 * try to reuse memory for sensor handle data
 */
static struct bme280_dev * m_bme280_reuse_mem(struct bme280_dev * const opt_BME280_dev)
{
   struct bme280_dev * pt_BME280_dev = opt_BME280_dev;

   if (pt_BME280_dev != NULL)
   {
      // check for valid handle data
      if ((pt_BME280_dev->intf == BME280_I2C_INTF) &&
          (pt_BME280_dev->read == &m_bme280_i2c_read) &&
          (pt_BME280_dev->write == &m_bme280_i2c_write) &&
          (pt_BME280_dev->delay_us == &m_bme280_delay_us))
      {
         if (pt_BME280_dev->intf_ptr != NULL)
         {
            i2c_master_bus_rm_device(pt_BME280_dev->intf_ptr);
         }

         // reuse memory: clear data
         (void)memset(pt_BME280_dev, 0, sizeof(struct bme280_dev));
      }
      else
      {
         // unknown handle data: can't be used, return NULL
         pt_BME280_dev = NULL;
      }
   }

   return pt_BME280_dev;
}

/*!
 * sensor API: init specific BME280 device
 */
static esp_err_t m_bme280_sensor_init(const T_sensor_def * const opt_SensorDef)
{
   const T_sensor_info * const pt_SensorInfo = &opt_SensorDef->t_SensorInfo;
   T_sensor_instance * const pt_SensorInstance = opt_SensorDef->pt_SensorInstance;   
   esp_err_t t_Err = m_bme280_get_mem(pt_SensorInstance);

   if (t_Err == ESP_OK)
   {
      t_Err = iic_mux_set(pt_SensorInfo->u8_IicBus);
      if (t_Err == ESP_OK)
      {
         struct bme280_dev * const pt_BME280_dev = pt_SensorInstance->pv_Handle;   
         t_Err = ESP_FAIL;

         if (m_bme280_interface_init(pt_SensorInfo->u8_IicAddr, pt_SensorInfo->u32_IicSclFreq, pt_BME280_dev) == ESP_OK)
         {
            // BME280 device init (see BOSCH sensor API example code)
            // init the device, read calib data etc.
            if (bme280_init(pt_BME280_dev) == BME280_OK)
            {
               struct bme280_settings settings;

               /* Always read the current settings before writing, especially when all the configuration is not modified */
               if (bme280_get_sensor_settings(&settings, pt_BME280_dev) == BME280_OK)
               {
                  /* Configuring the over-sampling rate, filter coefficient and standby time */
                  /* Overwrite the desired settings */
                  /* Over-sampling rate for humidity, temperature and pressure */
                  settings.osr_h = BME280_OVERSAMPLING_4X;
                  settings.osr_p = BME280_OVERSAMPLING_4X;
                  settings.osr_t = BME280_OVERSAMPLING_1X;
                  settings.filter = BME280_FILTER_COEFF_2;
                  /* Setting the standby time */
                  settings.standby_time = BME280_STANDBY_TIME_500_MS;

                  if (bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, pt_BME280_dev) == BME280_OK)
                  {
                     /* enter normal power mode: automatic data sampling with above settings */
                     if (bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, pt_BME280_dev) == BME280_OK)
                     {
                        /* Info: Calculate measurement time in microseconds */
                        /*
                        uint32_t u32_Period;
                        bme280_cal_meas_delay(&u32_Period, &settings);
                        printf("BME280 initialized, measurement time is %lu us\r\n", u32_Period);
                        */

                        // setup process timer
                        pt_SensorInstance->u64_TimeStampUs = esp_timer_get_time();
                        pt_SensorInstance->u32_NextCallUs = opt_SensorDef->u32_UpdateRateUs;
                        t_Err = ESP_OK;
                     }
                  }
               }
            }
         }
      }

      if (t_Err != ESP_OK)
      {
         m_bme280_free_mem(pt_SensorInstance); // on error: free all allocated resources
      }
   }

   return t_Err;
}

/*!
 * BME280 process data task
 */
static esp_err_t m_bme280_sensor_process_data(const T_sensor_def * const opt_SensorDef)
{
   esp_err_t t_Err = ESP_ERR_NOT_FINISHED;
   T_sensor_instance * const pt_SensorInstance = opt_SensorDef->pt_SensorInstance;   
   // get system time
   const uint64_t u64_SysTime = esp_timer_get_time();

   if (u64_SysTime >= pt_SensorInstance->u64_TimeStampUs + pt_SensorInstance->u32_NextCallUs)
   {
      // if time elapsed -> get new sensor data
      T_sensor_data * const pt_SensorData = &pt_SensorInstance->t_SensorData;
      t_Err = ESP_FAIL;

      if (iic_mux_set(opt_SensorDef->t_SensorInfo.u8_IicBus) == ESP_OK)
      {
         struct bme280_dev * const pt_BME280_dev = pt_SensorInstance->pv_Handle;   
         struct bme280_data t_SensorData;

         if (bme280_get_sensor_data(BME280_ALL, &t_SensorData, pt_BME280_dev) == BME280_OK)
         {
            pt_SensorData->f32_Temperature = (float)t_SensorData.temperature * 0.01f;
            pt_SensorData->f32_Pressure = (float)t_SensorData.pressure * 0.01f;
            pt_SensorData->f32_Humidity = (float)t_SensorData.humidity * 0.001f;
            pt_SensorData->f32_CO2 = SENSOR_VALUE_INVALID;
            pt_SensorData->f32_IAQ = SENSOR_VALUE_INVALID;
            pt_SensorData->u32_AgeSec = 0u;

            pt_SensorInstance->u64_TimeStampUs = u64_SysTime;
            pt_SensorInstance->u32_NextCallUs = opt_SensorDef->u32_UpdateRateUs;
            t_Err = ESP_OK;
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
 * get all BME280 sensor data
 */
static esp_err_t m_bme280_sensor_get_data(const T_sensor_def * const opt_SensorDef, T_sensor_data * const opt_SensorData)
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
static BME280_INTF_RET_TYPE m_bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
   BME280_INTF_RET_TYPE t_Result = BME280_INTF_RET_SUCCESS;

   if (i2c_master_transmit_receive(intf_ptr, &reg_addr, 1u, reg_data, length, BME280_IIC_TIMEOUT_MS) != ESP_OK)
   {
      t_Result = BME280_E_COMM_FAIL;
   }

   return t_Result;
}

/*!
 * I2C write function map to ESP32 platform
 */
static BME280_INTF_RET_TYPE m_bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
   uint8_t au8_temp_buff[BME280_WRITE_REGS_MAX*2u];
   BME280_INTF_RET_TYPE t_Result = BME280_OK;

   if (length < (BME280_WRITE_REGS_MAX*2u))
   {
      // prepare temp buffer
      au8_temp_buff[0] = reg_addr;
      memcpy(&au8_temp_buff[1], reg_data, length);
      
      if (i2c_master_transmit(intf_ptr, au8_temp_buff, length+1u, BME280_IIC_TIMEOUT_MS) != ESP_OK)
      {
         t_Result = BME280_E_COMM_FAIL;
      }
   }
   else
   {
      t_Result = BME280_E_INVALID_LEN;
   }

   return t_Result;
}

/*!
 * BOSCH HAL delay function
 */
static void m_bme280_delay_us(uint32_t u32_delay_us, void *intf_ptr)
{
   (void)intf_ptr;
   delay_us(u32_delay_us);
}