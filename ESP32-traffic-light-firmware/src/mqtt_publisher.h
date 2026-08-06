#pragma once

#include <stdbool.h>

#include "v2x_messages.h"

typedef enum {
    MQTT_LIGHT_COMMAND_TRAFFIC,
    MQTT_LIGHT_COMMAND_NORTH_SOUTH,
    MQTT_LIGHT_COMMAND_EAST_WEST,
} mqtt_light_command_t;

/*
 * Starts Wi-Fi and MQTT when MQTT_WIFI_SSID is configured. map_message must
 * remain valid for the life of the application; it is published retained
 * whenever the broker connection is established.
 */
void mqtt_publisher_init(const char *map_message);
bool mqtt_publisher_take_command(mqtt_light_command_t *command);
bool mqtt_publisher_take_srm(v2x_srm_request_t *request);
void mqtt_publisher_publish_spat(const char *spat_message);
void mqtt_publisher_publish_ssm(const char *ssm_message);
void mqtt_publisher_set_status(const char *status);
