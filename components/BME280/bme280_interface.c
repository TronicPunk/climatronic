//----------------------------------------------------------------------------------------------------------------------
/*!
   \file    bme280_interface.c
   \brief   BME280 interface: Glue layer between BOSCH Sensor API and ESP32 driver library
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <string.h>
#include "bme280_interface.h"
#include "sensor_name.h"
#include "iic.h"
#include "iic_mux.h"
#include "timer.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define BME280_IIC_SCL_TIMEOUT_US   (10000u)    // IIC SCL timeout in us
#define BME280_IIC_TIMEOUT_MS       (50u)       // IIC transaction timeout in ms

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static esp_err_t m_bme280_get_mem(T_sensor_instance ** opt_Sensor);
static struct bme280_dev * m_bme280_reuse_mem(struct bme280_dev * const opt_BME280_dev);
static void m_bme280_free_mem(T_sensor_instance * const opt_Sensor);
static esp_err_t m_bme280_interface_init(const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                         struct bme280_dev * opt_BME280_dev);

// common sensor API
static esp_err_t m_bme280_sensor_init(const uint8_t ou8_IicBus, const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                      T_sensor_instance ** oppt_Sensor);
static esp_err_t m_bme280_sensor_process_data(T_sensor_instance * const opt_Sensor);
static esp_err_t m_bme280_sensor_get_data(T_sensor_instance * const opt_Sensor, T_sensor_data * const opt_SensorData);
static esp_err_t m_bme280_sensor_command(T_sensor_instance * const opt_Sensor, const uint16_t ou16_Command, uint32_t * const  opu32_Parameter);

// low level driver functions
static BME280_INTF_RET_TYPE m_bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
static BME280_INTF_RET_TYPE m_bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
static void m_bme280_delay_us(uint32_t u32_delay_us, void *intf_ptr);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static const T_sensor_type mt_BME280_type =
{
   .s_Type = "BME280",
   .u16_SensorCaps = ((SENSOR_CAP_TEMPERATURE|SENSOR_CAP_PRESSURE|SENSOR_CAP_HUMIDITY) |\
                      (SENSOR_CAP_MODE_SLEEP|SENSOR_CAP_MODE_SINGLE|SENSOR_CAP_MODE_CYCLIC))
};

// delivers common sensor API
static const T_sensor_api mt_BME280_api =
{
   .pr_SensorInit = &m_bme280_sensor_init,
   .pr_SensorProcessData = &m_bme280_sensor_process_data,
   .pr_SensorGetData = &m_bme280_sensor_get_data,
   .pr_SensorCommand = &m_bme280_sensor_command
};

// complete sensor description
const T_sensor_descriptor gt_BME280_descriptor =
{
   .pt_Sensor = &mt_BME280_type,
   .pt_SensorApi = &mt_BME280_api,
   .u32_UpdateRateMinUs = 100000u,
   .u32_IicFreqMaxHz = 400000u,
   .u8_IicAddrCount = 1u,
   .au8_IicAddr = { BME280_I2C_ADDR_PRIM, BME280_I2C_ADDR_SEC }
};

/* -- Implementation ------------------------------------------------------------------------------------------------ */

void __force_link_bme280(void)
{
}

/**
 * allocate memory for sensor instance data or reuse already allocated memory
 */
static esp_err_t m_bme280_get_mem(T_sensor_instance ** oppt_Sensor)
{
   esp_err_t t_Err = ESP_OK;
   T_sensor_instance * pt_Sensor = *oppt_Sensor;

   if (pt_Sensor != NULL)
   {
      // check for valid sensor instance data
      if (pt_Sensor->pt_SensorDescriptor == &gt_BME280_descriptor)
      {
         // try to reuse memory for driver data:
         // return NULL if data not valid - otherwise pointer to empty memory block
         pt_Sensor->pv_Handle = m_bme280_reuse_mem(pt_Sensor->pv_Handle);
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
         void * pv_Handle = calloc(1, sizeof(struct bme280_dev));
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
      m_bme280_free_mem(pt_Sensor);
      pt_Sensor = NULL;
   }

   *oppt_Sensor = pt_Sensor;

   return t_Err;
}

/**
 * free memory and IIC device
 */
static void m_bme280_free_mem(T_sensor_instance * const opt_Sensor)
{
   if (opt_Sensor != NULL)
   {
      // this will check for valid handle data and also free any IIC device
      (void)m_bme280_reuse_mem(opt_Sensor->pv_Handle);

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

/**
 * init the BME280 interface (IIC)
 */
static esp_err_t m_bme280_interface_init(const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                         struct bme280_dev* opt_BME280_dev)
{
   esp_err_t t_Err = ESP_FAIL;
   i2c_master_bus_handle_t t_IicBusHandle = iic_get_bus_handle();

   if ((t_IicBusHandle != NULL) && (opt_BME280_dev != NULL))
   {
      // create new IIC sensor device
      i2c_device_config_t t_IicDev =
      {
         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
         .device_address = ou8_IicAddr,
         .scl_speed_hz = ou32_IicFreq,
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

/*!
 * sensor API: init specific BME280 device
 */
static esp_err_t m_bme280_sensor_init(const uint8_t ou8_IicBus, const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                      T_sensor_instance ** oppt_Sensor)
{
   T_sensor_instance * pt_Sensor = *oppt_Sensor;
   esp_err_t t_Err = m_bme280_get_mem(&pt_Sensor);

   if (t_Err == ESP_OK)
   {
      t_Err = iic_mux_set(ou8_IicBus);
      if (t_Err == ESP_OK)
      {
         struct bme280_dev * const pt_BME280_dev = pt_Sensor->pv_Handle;
         t_Err = ESP_FAIL;

         if (m_bme280_interface_init(ou8_IicAddr, ou32_IicFreq, pt_BME280_dev) == ESP_OK)
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
//                        uint32_t u32_Period;
//                        bme280_cal_meas_delay(&u32_Period, &settings);

                        // setup sensor instance data
                        pt_Sensor->pt_SensorDescriptor = &gt_BME280_descriptor;
                        pt_Sensor->u32_IicFreq = ou32_IicFreq;
                        pt_Sensor->u8_IicBus = ou8_IicBus;
                        pt_Sensor->u8_IicAddr = ou8_IicAddr;
                        pt_Sensor->s_SensorName = sensor_name_create(pt_Sensor);
                        // setup process timer
                        pt_Sensor->u64_CallTimeUs = esp_timer_get_time() + pt_Sensor->pt_SensorDescriptor->u32_UpdateRateMinUs;;
                        t_Err = ESP_OK;
                     }
                  }
               }
            }
         }
      }

      if (t_Err != ESP_OK)
      {
         m_bme280_free_mem(pt_Sensor); // on error: free all allocated resources
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
 * BME280 process data task
 */
static esp_err_t m_bme280_sensor_process_data(T_sensor_instance * const opt_Sensor)
{
   esp_err_t t_Err = ESP_ERR_NOT_FINISHED;
   // get system time
   const uint64_t u64_SysTime = esp_timer_get_time();

   if (u64_SysTime >= opt_Sensor->u64_CallTimeUs)
   {
      t_Err = ESP_FAIL;
      // if time elapsed -> get new sensor data
      if (iic_mux_set(opt_Sensor->u8_IicBus) == ESP_OK)
      {
         struct bme280_dev * const pt_BME280_dev = opt_Sensor->pv_Handle;
         struct bme280_data t_SensorData;

         if (bme280_get_sensor_data(BME280_ALL, &t_SensorData, pt_BME280_dev) == BME280_OK)
         {
            T_sensor_data * const pt_SensorData = &opt_Sensor->t_SensorData;

            pt_SensorData->f32_Temperature = (float)t_SensorData.temperature * 0.01f;
            pt_SensorData->f32_Pressure = (float)t_SensorData.pressure * 0.0001f;
            pt_SensorData->f32_Humidity = (float)t_SensorData.humidity * 0.001f;
            pt_SensorData->f32_CO2 = SENSOR_VALUE_INVALID;
            pt_SensorData->f32_IAQ = SENSOR_VALUE_INVALID;
            pt_SensorData->u32_TimeStampSec = get_time_sec();

            opt_Sensor->u64_CallTimeUs = u64_SysTime + opt_Sensor->pt_SensorDescriptor->u32_UpdateRateMinUs;
            t_Err = ESP_OK;
         }
      }
   }

   return t_Err;
}

/*!
 * get all BME280 sensor data
 */
static esp_err_t m_bme280_sensor_get_data(T_sensor_instance * const opt_Sensor, T_sensor_data * const opt_SensorData)
{
   *opt_SensorData = opt_Sensor->t_SensorData;
   // at least one data sample available? -> return ESP_OK
   return (opt_Sensor->t_SensorData.u32_TimeStampSec == SENSOR_TIMESTAMP_INVALID) ? ESP_ERR_NOT_FINISHED : ESP_OK;
}

/*!
 * BME280 do not support any commands
 */
static esp_err_t m_bme280_sensor_command(T_sensor_instance * const opt_Sensor, const uint16_t ou16_Command, uint32_t * const  opu32_Parameter)
{
   esp_err_t t_Err = ESP_ERR_NOT_SUPPORTED;

   if (ou16_Command == SENSOR_COMMAND_GET_STATE)
   {
      *opu32_Parameter = opt_Sensor->u32_Parameter;
      t_Err = ESP_OK;
   }

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