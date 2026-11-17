#ifndef REMOTE_MQTT_H
#define REMOTE_MQTT_H

#include "mqtt_data.h"
#include "esp_err.h"

esp_err_t remote_mqtt_start(T_mqtt_data * const opt_MqttData);
void remote_mqtt_stop(void);

#endif
