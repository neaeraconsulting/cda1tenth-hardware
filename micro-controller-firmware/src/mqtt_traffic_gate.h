#ifndef MQTT_TRAFFIC_GATE_H
#define MQTT_TRAFFIC_GATE_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "mqtt_demo_config.h"

enum class TrafficMovementState : uint8_t
{
  Unknown,
  Stop,
  Clearance,
  Allowed
};

class MqttTrafficGate
{
public:
  MqttTrafficGate();

  void begin();
  void loop();

  bool isEnabled() const;
  bool isConnected();
  bool hasFreshSpat() const;
  float speedMultiplier() const;
  const char *movementStateName() const;
  int signalGroup() const;
  int timeRemaining() const;
  void setSignalGroup(int signal_group);
  void handleMessage(char *topic, const uint8_t *payload, unsigned int length);

private:
  WiFiClient wifi_client_;
  PubSubClient mqtt_client_;
  bool enabled_ = false;
  bool wifi_started_ = false;
  bool wifi_configured_ = false;
  bool start_logged_ = false;
  bool subscriptions_logged_ = false;
  bool last_wifi_connected_ = false;
  bool last_mqtt_connected_ = false;
  bool map_logged_ = false;
  bool movement_logged_ = false;
  bool stale_logged_ = false;
  unsigned long last_wifi_attempt_ms_ = 0;
  unsigned long last_mqtt_attempt_ms_ = 0;
  unsigned long boot_ms_ = 0;
  unsigned long last_spat_ms_ = 0;
  unsigned long movement_state_started_ms_ = 0;
  TrafficMovementState movement_state_ = TrafficMovementState::Unknown;
  TrafficMovementState logged_movement_state_ = TrafficMovementState::Unknown;
  int signal_group_ = MQTT_VEHICLE_SIGNAL_GROUP;
  int time_remaining_ = -1;
  char client_id_[40] = {0};

  void ensureWifi(unsigned long now_ms);
  void ensureMqtt(unsigned long now_ms);
  void logConnectionChanges();
  void logStaleState();
  void subscribeTopics();
  void handleSpat(const char *payload);
  void handleMap(const char *payload);
};

#endif // MQTT_TRAFFIC_GATE_H
