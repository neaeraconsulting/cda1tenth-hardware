#pragma once

// Copy this file to secrets.h and enter the Wi-Fi used by the MQTT broker.
#define MQTT_WIFI_SSID "your-wifi-ssid"
#define MQTT_WIFI_PASSWORD "your-wifi-password"

// Optional overrides. The shared anonymous MQTT broker defaults are shown here.
// #define MQTT_BROKER_HOST "172.250.250.111"
// #define MQTT_BROKER_PORT 1883

// Optional topic overrides.
// #define MQTT_SPAT_TOPIC "esp32/1/spat"
// #define MQTT_MAP_TOPIC "esp32/1/map"
// #define MQTT_COMMAND_TOPIC "esp32/1/color"
// #define MQTT_STATUS_TOPIC "esp32/1/status/color"
// #define MQTT_SRM_TOPIC "esp32/1/srm"
// #define MQTT_SSM_TOPIC "esp32/1/ssm"

// Four-byte J2735 TemporaryID, represented as eight hexadecimal characters.
// This is a model-scale allowlist, not cryptographic V2X authentication.
// #define V2X_AUTHORIZED_ENTITY_ID "01020304"

// Optional UTC time source used for J2735 timing fields.
// #define SNTP_SERVER "pool.ntp.org"
