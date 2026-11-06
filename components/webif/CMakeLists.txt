idf_component_register(
   SRCS
      "wifi_connect.c"
      "webserver.c"
   INCLUDE_DIRS
      "."
   PRIV_REQUIRES
      esp_wifi
      esp_http_server
      esp_timer
      nvs_flash
      mdns
      sensor
      output
      system_data
)

# HTML-Datei einbinden (als binary)
target_add_binary_data(${COMPONENT_TARGET} "index.html" TEXT)
