#pragma once

// Template for include/secrets.h with your demo WiFi.
// include/secrets.h is ignored by git.
#define MQTT_WIFI_SSID "your-wifi-ssid"
#define MQTT_WIFI_PASSWORD "your-wifi-password"

// Optional overrides. The defaults target a broker on the Windows Wi-Fi
// adapter at 10.0.0.92 and the single-signal SPaT topic esp32/1/spat.
// #define MQTT_BROKER_HOST "10.0.0.92"
// #define MQTT_VEHICLE_SIGNAL_GROUP 1
