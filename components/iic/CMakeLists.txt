idf_component_register(
   SRCS
      "iic.c"
   INCLUDE_DIRS
      "."
   REQUIRES
      driver
      esp_driver_i2c

)
