idf_component_register(
   SRCS
      "firmware_main.c" "serial.c" "menu.c" "debug.c"
   INCLUDE_DIRS
      "."
   PRIV_REQUIRES
      nvs_flash
      driver
      iic
      iic_mux
      sensor
      BME280
      BME68x
      SCD4x
      output
      webif
      timer
)
