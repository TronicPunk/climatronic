#ifndef MQTT_INTERFACE_H
#define MQTT_INTERFACE_H

#include "mqtt_data.h"
#include "esp_err.h"
#include <stdbool.h>

esp_err_t mqtt_client_start(T_mqtt_data * const opt_MqttData);
esp_err_t mqtt_client_restart(void);
bool mqtt_client_is_connected(void);
void mqtt_client_stop(void);

#endif // MQTT_INTERFACE_H
