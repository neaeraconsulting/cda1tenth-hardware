#pragma once

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef MQTT_WIFI_SSID
#define MQTT_WIFI_SSID ""
#endif

#ifndef MQTT_WIFI_PASSWORD
#define MQTT_WIFI_PASSWORD ""
#endif

#ifndef MQTT_BROKER_HOST
#define MQTT_BROKER_HOST "172.250.250.111"
#endif

#ifndef MQTT_BROKER_PORT
#define MQTT_BROKER_PORT 1883
#endif

#ifndef MQTT_SPAT_TOPIC
#define MQTT_SPAT_TOPIC "esp32/1/spat"
#endif

#ifndef MQTT_MAP_TOPIC
#define MQTT_MAP_TOPIC "esp32/1/map"
#endif

#ifndef MQTT_COMMAND_TOPIC
#define MQTT_COMMAND_TOPIC "esp32/1/color"
#endif

#ifndef MQTT_STATUS_TOPIC
#define MQTT_STATUS_TOPIC "esp32/1/status/color"
#endif

#ifndef MQTT_SRM_TOPIC
#define MQTT_SRM_TOPIC "esp32/1/srm"
#endif

#ifndef MQTT_SSM_TOPIC
#define MQTT_SSM_TOPIC "esp32/1/ssm"
#endif

/*
 * Model-scale authorization filter for J2735 SRM requestor entity IDs.
 * This is not a substitute for signed V2X messages and SCMS validation.
 */
#ifndef V2X_AUTHORIZED_ENTITY_ID
#define V2X_AUTHORIZED_ENTITY_ID "01020304"
#endif

#ifndef MQTT_BUFFER_SIZE
#define MQTT_BUFFER_SIZE 3072
#endif

#ifndef MQTT_OUTBOX_LIMIT_BYTES
#define MQTT_OUTBOX_LIMIT_BYTES 32768
#endif

#ifndef SNTP_SERVER
#define SNTP_SERVER "pool.ntp.org"
#endif
