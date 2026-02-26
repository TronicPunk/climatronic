//----------------------------------------------------------------------------------------------------------------------
/*!
   \file    bme68x_interface.c
   \brief   BME68x interface: Glue layer between BOSCH Sensor API and ESP32 driver library
*/
//----------------------------------------------------------------------------------------------------------------------

/* -- Includes ------------------------------------------------------------------------------------------------------ */
#include <string.h>
#include "bme68x_interface.h"
#include "sensor_name.h"
#include "iic.h"
#include "iic_mux.h"
#include "timer.h"

/* -- Defines ------------------------------------------------------------------------------------------------------- */
#define BME68X_IIC_SCL_TIMEOUT_US   (10000u)    // IIC SCL timeout in us
#define BME68X_IIC_TIMEOUT_MS       (50u)       // IIC transaction timeout in ms
#define BME68X_STATE_OPERATIONAL    (0u)        // normal operation, process sensor data
#define BME68X_STATE_MEASURE        (1u)        // trigger next measurement

/* -- Types --------------------------------------------------------------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------------------------------------------------- */

static esp_err_t m_bme68x_get_mem(T_sensor_instance ** const oppt_Sensor);
static struct bme68x_dev * m_bme68x_reuse_mem(struct bme68x_dev * const opt_BME68x_dev);
static void m_bme68x_free_mem(T_sensor_instance * const opt_Sensor);
static esp_err_t m_bme68x_interface_init(const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                         struct bme68x_dev * opt_BME68x_dev);

// common sensor API
static esp_err_t m_bme68x_sensor_init(const uint8_t ou8_IicBus, const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                      T_sensor_instance ** oppt_Sensor);
static esp_err_t m_bme68x_sensor_process_data(T_sensor_instance * const opt_Sensor);
static esp_err_t m_bme68x_sensor_get_data(T_sensor_instance * const opt_Sensor, T_sensor_data * const opt_SensorData);
static esp_err_t m_bme68x_sensor_command(T_sensor_instance * const opt_Sensor, const uint16_t ou16_Command, uint32_t * const  opu32_Parameter);

// low level driver functions
static BME68X_INTF_RET_TYPE m_bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
static BME68X_INTF_RET_TYPE m_bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
static void m_bme68x_delay_us(uint32_t u32_delay_us, void *intf_ptr);

/* -- (Module) Global Variables ------------------------------------------------------------------------------------- */

static const T_sensor_type mt_BME680_type =
{
   .s_Type = "BME680",
   .u16_SensorCaps = ((SENSOR_CAP_TEMPERATURE|SENSOR_CAP_PRESSURE|SENSOR_CAP_HUMIDITY|SENSOR_CAP_IAQ) |\
                      (SENSOR_CAP_MODE_SLEEP|SENSOR_CAP_MODE_SINGLE))
};

// delivers common sensor API
static const T_sensor_api mt_BME68x_api =
{
   .pr_SensorInit = &m_bme68x_sensor_init,
   .pr_SensorProcessData = &m_bme68x_sensor_process_data,
   .pr_SensorGetData = &m_bme68x_sensor_get_data,
   .pr_SensorCommand = &m_bme68x_sensor_command
};

// complete sensor description
const T_sensor_descriptor gt_BME680_descriptor =
{
   .pt_Sensor = &mt_BME680_type,
   .pt_SensorApi = &mt_BME68x_api,
   .u32_UpdateRateMinUs = 3000000u,
   .u32_IicFreqMaxHz = 400000u,
   .u8_IicAddrCount = 1u,
   .au8_IicAddr = { BME68X_I2C_ADDR_HIGH, BME68X_I2C_ADDR_LOW }
};

/* -- Implementation ------------------------------------------------------------------------------------------------ */

void __force_link_bme68x(void)
{
}

/**
 * allocate memory for sensor instance data or reuse already allocated memory
 */
static esp_err_t m_bme68x_get_mem(T_sensor_instance ** oppt_Sensor)
{
   esp_err_t t_Err = ESP_OK;
   T_sensor_instance * pt_Sensor = *oppt_Sensor;

   if (pt_Sensor != NULL)
   {
      // check for valid sensor instance data
      if (pt_Sensor->pt_SensorDescriptor == &gt_BME680_descriptor)
      {
         // try to reuse memory for driver data:
         // return NULL if data not valid - otherwise pointer to empty memory block
         pt_Sensor->pv_Handle = m_bme68x_reuse_mem(pt_Sensor->pv_Handle);
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
         void * pv_Handle = calloc(1, sizeof(struct bme68x_dev));
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
      m_bme68x_free_mem(pt_Sensor);
      pt_Sensor = NULL;
   }

   *oppt_Sensor = pt_Sensor;

   return t_Err;
}

/**
 * free memory and IIC device
 */
static void m_bme68x_free_mem(T_sensor_instance * const opt_Sensor)
{
   if (opt_Sensor != NULL)
   {
      // this will check for valid handle data and also free any IIC device
      (void)m_bme68x_reuse_mem(opt_Sensor->pv_Handle);

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
static struct bme68x_dev * m_bme68x_reuse_mem(struct bme68x_dev * const opt_BME68x_dev)
{
   struct bme68x_dev * pt_BME68x_dev = opt_BME68x_dev;

   if (pt_BME68x_dev != NULL)
   {
      // check for valid handle data
      if ((pt_BME68x_dev->intf == BME68X_I2C_INTF) &&
          (pt_BME68x_dev->read == &m_bme68x_i2c_read) &&
          (pt_BME68x_dev->write == &m_bme68x_i2c_write) &&
          (pt_BME68x_dev->delay_us == &m_bme68x_delay_us))
      {
         if (pt_BME68x_dev->intf_ptr != NULL)
         {
            i2c_master_bus_rm_device(pt_BME68x_dev->intf_ptr);
         }

         // reuse memory: clear data
         (void)memset(pt_BME68x_dev, 0, sizeof(struct bme68x_dev));
      }
      else
      {
         // unknown handle data: can't be used, return NULL
         pt_BME68x_dev = NULL;
      }
   }

   return pt_BME68x_dev;
}

/**
 * init the BME68x interface (IIC)
 */
static esp_err_t m_bme68x_interface_init(const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                         struct bme68x_dev* opt_BME68x_dev)
{
   esp_err_t t_Err = ESP_FAIL;
   i2c_master_bus_handle_t t_IicBusHandle = iic_get_bus_handle();

   if ((t_IicBusHandle != NULL) && (opt_BME68x_dev != NULL))
   {
      // create new IIC sensor device
      i2c_device_config_t t_IicDev =
      {
         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
         .device_address = ou8_IicAddr,
         .scl_speed_hz = ou32_IicFreq,
         .scl_wait_us = BME68X_IIC_SCL_TIMEOUT_US,
         .flags.disable_ack_check = false
      };
      i2c_master_dev_handle_t t_IicDevHandle = NULL;

      t_Err = i2c_master_bus_add_device(t_IicBusHandle, &t_IicDev, &t_IicDevHandle);
      ESP_ERROR_CHECK(t_Err);

      if (t_Err == ESP_OK)
      {
         // register mt_BME68x_dev
         opt_BME68x_dev->intf = BME68X_I2C_INTF;
         opt_BME68x_dev->intf_ptr = t_IicDevHandle;
         opt_BME68x_dev->read = &m_bme68x_i2c_read;
         opt_BME68x_dev->write = &m_bme68x_i2c_write;
         opt_BME68x_dev->delay_us = &m_bme68x_delay_us;
      }
   }

   return t_Err;
}

/*!
 * sensor API: init specific BME68x device
 */
static esp_err_t m_bme68x_sensor_init(const uint8_t ou8_IicBus, const uint8_t ou8_IicAddr, const uint32_t ou32_IicFreq,
                                      T_sensor_instance ** oppt_Sensor)
{
   T_sensor_instance * pt_Sensor = *oppt_Sensor;
   esp_err_t t_Err = m_bme68x_get_mem(&pt_Sensor);

   if (t_Err == ESP_OK)
   {
      t_Err = iic_mux_set(ou8_IicBus);
      if (t_Err == ESP_OK)
      {
         struct bme68x_dev * pt_BME68x_dev = pt_Sensor->pv_Handle;
         t_Err = ESP_FAIL;

         if (m_bme68x_interface_init(ou8_IicAddr, ou32_IicFreq, pt_BME68x_dev) == ESP_OK)
         {
            // BME68x device init (see BOSCH sensor API example code)
            // init the device, read calib data etc.
            if (bme68x_init(pt_BME68x_dev) == BME68X_OK)
            {
               struct bme68x_conf conf;

               /* Always read the current settings before writing, especially when all the configuration is not modified */
               if (bme68x_get_conf(&conf, pt_BME68x_dev) == BME68X_OK)
               {
                  /* Configuring the over-sampling rate, filter coefficient and standby time */
                  /* Overwrite the desired settings */
                  /* Over-sampling rate for humidity, temperature and pressure */
                  conf.os_hum = BME68X_OS_16X;
                  conf.os_pres = BME68X_OS_1X;
                  conf.os_temp = BME68X_OS_2X;
                  conf.filter = BME68X_FILTER_OFF;
                  /* Setting the standby time */
                  conf.odr = BME68X_ODR_500_MS;

                  if (bme68x_set_conf(&conf, pt_BME68x_dev) == BME68X_OK)
                  {
                     struct bme68x_heatr_conf heatr_conf;

                     /* Configuring the gas sensor heater */
                     heatr_conf.enable = BME68X_ENABLE;
                     heatr_conf.heatr_temp = 300;
                     heatr_conf.heatr_dur = 100;
                     heatr_conf.heatr_temp_prof = NULL;
                     heatr_conf.heatr_dur_prof = NULL;
                     heatr_conf.profile_len = 0;
                     heatr_conf.shared_heatr_dur = 0;

                     if (bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, pt_BME68x_dev) == BME68X_OK)
                     {
                        /* enter sleep mode */
                        if (bme68x_set_op_mode(BME68X_FORCED_MODE, pt_BME68x_dev) == BME68X_OK)
                        {
                           /* Info: Calculate measurement time in microseconds (result for this setup is < 150ms)*/
                           /*
                           const uint32_t u32_Period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, pt_BME68x_dev) + (heatr_conf.heatr_dur * 1000);
                           printf("BME68x initialized, measurement time is %lu us\r\n", u32_Period);
                           */

                           // setup sensor instance data
                           pt_Sensor->pt_SensorDescriptor = &gt_BME680_descriptor;
                           pt_Sensor->u32_IicFreq = ou32_IicFreq;
                           pt_Sensor->u8_IicBus = ou8_IicBus;
                           pt_Sensor->u8_IicAddr = ou8_IicAddr;
                           pt_Sensor->s_SensorName = sensor_name_create(pt_Sensor);
                           // setup process timer: 1st data sample should be available after 0.2s
                           pt_Sensor->u64_CallTimeUs = esp_timer_get_time() + 200000uL;
                           pt_Sensor->u16_SensorState = BME68X_STATE_OPERATIONAL;
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
         m_bme68x_free_mem(pt_Sensor); // on error: free all allocated resources
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
 * BME68x process data task
 */
static esp_err_t m_bme68x_sensor_process_data(T_sensor_instance * const opt_Sensor)
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
         struct bme68x_dev * const pt_BME68x_dev = opt_Sensor->pv_Handle;

         if (opt_Sensor->u16_SensorState == BME68X_STATE_OPERATIONAL)
         {
            struct bme68x_data t_SensorData;
            uint8_t n_fields;

            if (bme68x_get_data(BME68X_FORCED_MODE, &t_SensorData, &n_fields, pt_BME68x_dev) == BME68X_OK)
            {
               if (n_fields != 0u)
               {
                  T_sensor_data * const pt_SensorData = &opt_Sensor->t_SensorData;

                  pt_SensorData->f32_Temperature = t_SensorData.temperature;
                  pt_SensorData->f32_Pressure = t_SensorData.pressure*0.01f; // pressure in hPa
                  pt_SensorData->f32_Humidity = t_SensorData.humidity;
                  pt_SensorData->f32_CO2 = SENSOR_VALUE_INVALID;
                  pt_SensorData->f32_IAQ = t_SensorData.gas_resistance;
                  pt_SensorData->u32_TimeStampSec = get_time_sec();

                  opt_Sensor->u16_SensorState = BME68X_STATE_MEASURE;
                  t_Err = ESP_OK;
               }
               else
               {
                  t_Err = ESP_ERR_NOT_FINISHED;
               }
            }
         }

         if (opt_Sensor->u16_SensorState == BME68X_STATE_MEASURE)
         {
            // now we have to trigger next measurement
            if (bme68x_set_op_mode(BME68X_FORCED_MODE, pt_BME68x_dev) == BME68X_OK)
            {
               opt_Sensor->u64_CallTimeUs = u64_SysTime + opt_Sensor->pt_SensorDescriptor->u32_UpdateRateMinUs;
               opt_Sensor->u16_SensorState = BME68X_STATE_OPERATIONAL;
               t_Err = ESP_OK;
            }
         }
      }
   }

   return t_Err;
}

/*!
 * get all BME68x sensor data
 */
static esp_err_t m_bme68x_sensor_get_data(T_sensor_instance * const opt_Sensor, T_sensor_data * const opt_SensorData)
{
   *opt_SensorData = opt_Sensor->t_SensorData;
   // at least one data sample available? -> return ESP_OK
   return (opt_Sensor->t_SensorData.u32_TimeStampSec == SENSOR_TIMESTAMP_INVALID) ? ESP_ERR_NOT_FINISHED : ESP_OK;
}

/*!
 * BME68x do not support any commands
 */
static esp_err_t m_bme68x_sensor_command(T_sensor_instance * const opt_Sensor, const uint16_t ou16_Command, uint32_t * const  opu32_Parameter)
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
static BME68X_INTF_RET_TYPE m_bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
   BME68X_INTF_RET_TYPE t_Result = BME68X_INTF_RET_SUCCESS;

   if (i2c_master_transmit_receive(intf_ptr, &reg_addr, 1u, reg_data, length, BME68X_IIC_TIMEOUT_MS) != ESP_OK)
   {
      t_Result = BME68X_E_COM_FAIL;
   }

   return t_Result;
}

/*!
 * I2C write function map to ESP32 platform
 */
static BME68X_INTF_RET_TYPE m_bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
   BME68X_INTF_RET_TYPE t_Result = BME68X_INTF_RET_SUCCESS;

   // note: BME68x API driver delivers reg_addr in preceding reg_data element
   if (i2c_master_transmit(intf_ptr, &reg_data[-1], length+1u, BME68X_IIC_TIMEOUT_MS) != ESP_OK)
   {
      t_Result = BME68X_E_COM_FAIL;
   }

   return t_Result;
}

/*!
 * BOSCH HAL delay function
 */
static void m_bme68x_delay_us(uint32_t u32_delay_us, void *intf_ptr)
{
   (void)intf_ptr;
   delay_us(u32_delay_us);
}