#include "mqtt_traffic_gate.h"

#include <ArduinoJson.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

namespace
{
MqttTrafficGate *active_gate = nullptr;

void mqttLog(const char *message)
{
#if MQTT_SERIAL_DEBUG
  Serial.println(message);
#else
  (void)message;
#endif
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  if (active_gate != nullptr)
  {
    active_gate->handleMessage(topic, payload, length);
  }
}

bool readInt(JsonVariantConst value, int &out)
{
  if (value.is<int>())
  {
    out = value.as<int>();
    return true;
  }

  if (value.is<const char *>())
  {
    char *end = nullptr;
    long parsed = strtol(value.as<const char *>(), &end, 10);
    if (end != value.as<const char *>())
    {
      out = static_cast<int>(parsed);
      return true;
    }
  }

  return false;
}

bool readObjectInt(JsonObjectConst object, const char *key, int &out)
{
  JsonVariantConst value = object[key];
  return !value.isNull() && readInt(value, out);
}

bool readSignalGroup(JsonObjectConst object, int &signal_group)
{
  return readObjectInt(object, "signalGroup", signal_group) ||
         readObjectInt(object, "signal_group", signal_group) ||
         readObjectInt(object, "group", signal_group);
}

bool readTimeRemaining(JsonObjectConst object, int &time_remaining)
{
  const char *keys[] = {"timeRemaining", "remainingTime", "timeLeft", "timeToChange", "minEndTime", "likelyTime"};

  for (const char *key : keys)
  {
    if (readObjectInt(object, key, time_remaining))
    {
      return true;
    }
  }

  JsonObjectConst timing = object["timing"];
  if (!timing.isNull())
  {
    for (const char *key : keys)
    {
      if (readObjectInt(timing, key, time_remaining))
      {
        return true;
      }
    }
  }

  return false;
}

TrafficMovementState parseEventState(const char *state)
{
  if (state == nullptr)
  {
    return TrafficMovementState::Unknown;
  }

  if (strcasecmp(state, "protected-Movement-Allowed") == 0 ||
      strcasecmp(state, "movement-allowed") == 0 ||
      strcasecmp(state, "green") == 0)
  {
    return TrafficMovementState::Allowed;
  }

  if (strcasecmp(state, "protected-clearance") == 0 ||
      strcasecmp(state, "clearance") == 0 ||
      strcasecmp(state, "yellow") == 0)
  {
    return TrafficMovementState::Clearance;
  }

  if (strcasecmp(state, "stop-And-Remain") == 0 ||
      strcasecmp(state, "stop") == 0 ||
      strcasecmp(state, "red") == 0 ||
      strcasecmp(state, "off") == 0)
  {
    return TrafficMovementState::Stop;
  }

  return TrafficMovementState::Unknown;
}

TrafficMovementState readEventState(JsonObjectConst event, int &time_remaining)
{
  const char *keys[] = {"eventState", "movementEventState", "signalState", "state", "color"};

  readTimeRemaining(event, time_remaining);

  for (const char *key : keys)
  {
    TrafficMovementState state = parseEventState(event[key]);
    if (state != TrafficMovementState::Unknown)
    {
      return state;
    }
  }

  return TrafficMovementState::Unknown;
}

TrafficMovementState readSignalState(JsonObjectConst signal_state, int &time_remaining)
{
  TrafficMovementState state = readEventState(signal_state, time_remaining);
  if (state != TrafficMovementState::Unknown)
  {
    return state;
  }

  const char *event_arrays[] = {"state-time-speed", "stateTimeSpeed", "events"};
  for (const char *key : event_arrays)
  {
    JsonArrayConst events = signal_state[key];
    if (events.isNull())
    {
      continue;
    }

    for (JsonObjectConst event : events)
    {
      state = readEventState(event, time_remaining);
      if (state != TrafficMovementState::Unknown)
      {
        return state;
      }
    }
  }

  return TrafficMovementState::Unknown;
}

bool parseSpatStates(JsonArrayConst states, int target_group, TrafficMovementState &movement_state, int &time_remaining)
{
  for (JsonObjectConst signal_state : states)
  {
    int signal_group = 0;
    if (!readSignalGroup(signal_state, signal_group) || signal_group != target_group)
    {
      continue;
    }

    TrafficMovementState parsed_state = readSignalState(signal_state, time_remaining);
    if (parsed_state != TrafficMovementState::Unknown)
    {
      movement_state = parsed_state;
      return true;
    }
  }

  return false;
}

bool parseSpat(JsonVariantConst root, int target_group, TrafficMovementState &movement_state, int &time_remaining)
{
  JsonObjectConst object = root.as<JsonObjectConst>();
  if (object.isNull())
  {
    return false;
  }

  JsonObjectConst value = object["value"];
  if (!value.isNull())
  {
    object = value;
  }

  JsonArrayConst states = object["states"];
  if (!states.isNull() && parseSpatStates(states, target_group, movement_state, time_remaining))
  {
    return true;
  }

  JsonArrayConst signal_groups = object["signalGroups"];
  if (!signal_groups.isNull() && parseSpatStates(signal_groups, target_group, movement_state, time_remaining))
  {
    return true;
  }

  JsonArrayConst intersections = object["intersections"];
  for (JsonObjectConst intersection : intersections)
  {
    states = intersection["states"];
    if (!states.isNull() && parseSpatStates(states, target_group, movement_state, time_remaining))
    {
      return true;
    }
  }

  int signal_group = 0;
  if (readSignalGroup(object, signal_group) && signal_group == target_group)
  {
    movement_state = readSignalState(object, time_remaining);
    return movement_state != TrafficMovementState::Unknown;
  }

  return false;
}

bool readLaneSignalGroup(JsonObjectConst lane, int &signal_group)
{
  if (readSignalGroup(lane, signal_group))
  {
    return true;
  }

  JsonArrayConst connects_to = lane["connectsTo"];
  for (JsonObjectConst connection : connects_to)
  {
    if (readSignalGroup(connection, signal_group))
    {
      return true;
    }
  }

  return false;
}

bool mapObjectMatchesVehicle(JsonObjectConst object)
{
#if MQTT_VEHICLE_LANE_ID > 0
  int lane_id = 0;
  if ((readObjectInt(object, "laneID", lane_id) ||
       readObjectInt(object, "laneId", lane_id) ||
       readObjectInt(object, "lane", lane_id)) &&
      lane_id == MQTT_VEHICLE_LANE_ID)
  {
    return true;
  }
#endif

  int ingress_approach = 0;
  if (readObjectInt(object, "ingressApproach", ingress_approach) &&
      ingress_approach == MQTT_VEHICLE_INGRESS_APPROACH)
  {
    return true;
  }

  const char *keys[] = {"approach", "approachName", "direction", "name"};
  for (const char *key : keys)
  {
    const char *value = object[key];
    if (value != nullptr && strcasecmp(value, MQTT_VEHICLE_APPROACH) == 0)
    {
      return true;
    }
  }

  return false;
}

bool parseMapLanes(JsonArrayConst lanes, int &signal_group)
{
  for (JsonObjectConst lane : lanes)
  {
    if (mapObjectMatchesVehicle(lane) && readLaneSignalGroup(lane, signal_group))
    {
      return true;
    }
  }

  return false;
}

bool parseMapLaneSet(JsonArrayConst lane_set, int &signal_group)
{
  int only_signal_group = 0;
  int signal_group_count = 0;

  for (JsonObjectConst lane : lane_set)
  {
    int lane_signal_group = 0;
    if (!readLaneSignalGroup(lane, lane_signal_group))
    {
      continue;
    }

    only_signal_group = lane_signal_group;
    ++signal_group_count;

    if (mapObjectMatchesVehicle(lane))
    {
      signal_group = lane_signal_group;
      return true;
    }
  }

  if (signal_group_count == 1)
  {
    signal_group = only_signal_group;
    return true;
  }

  return false;
}

bool parseMap(JsonVariantConst root, int &signal_group)
{
  JsonObjectConst object = root.as<JsonObjectConst>();
  if (object.isNull())
  {
    return false;
  }

  JsonObjectConst value = object["value"];
  if (!value.isNull())
  {
    object = value;
  }

  JsonArrayConst lanes = object["lanes"];
  if (!lanes.isNull() && parseMapLanes(lanes, signal_group))
  {
    return true;
  }

  JsonArrayConst lane_set = object["laneSet"];
  if (!lane_set.isNull() && parseMapLaneSet(lane_set, signal_group))
  {
    return true;
  }

  JsonArrayConst intersections = object["intersections"];
  for (JsonObjectConst intersection : intersections)
  {
    lanes = intersection["lanes"];
    if (!lanes.isNull() && parseMapLanes(lanes, signal_group))
    {
      return true;
    }

    lane_set = intersection["laneSet"];
    if (!lane_set.isNull() && parseMapLaneSet(lane_set, signal_group))
    {
      return true;
    }
  }

  if (mapObjectMatchesVehicle(object) && readLaneSignalGroup(object, signal_group))
  {
    return true;
  }

  return false;
}

const char *stateName(TrafficMovementState state)
{
  switch (state)
  {
  case TrafficMovementState::Allowed:
    return "green";
  case TrafficMovementState::Clearance:
    return "yellow";
  case TrafficMovementState::Stop:
    return "red";
  default:
    return "unknown";
  }
}
} // namespace

MqttTrafficGate::MqttTrafficGate()
    : mqtt_client_(wifi_client_)
{
}

void MqttTrafficGate::begin()
{
  enabled_ = strlen(MQTT_WIFI_SSID) > 0;
  boot_ms_ = millis();
  if (!enabled_)
  {
    mqttLog("MQTT disabled: MQTT_WIFI_SSID is empty");
    return;
  }

  active_gate = this;
  mqttLog("MQTT enabled: delayed WiFi start pending");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.persistent(false);

  snprintf(client_id_, sizeof(client_id_), "cda1-vehicle-%06X", static_cast<unsigned int>(ESP.getEfuseMac() & 0xFFFFFF));
  mqtt_client_.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  mqtt_client_.setCallback(mqttCallback);
  mqtt_client_.setBufferSize(MQTT_MESSAGE_BUFFER_BYTES);
}

void MqttTrafficGate::loop()
{
  if (!enabled_)
  {
    return;
  }

  unsigned long now = millis();
  if (now - boot_ms_ < MQTT_START_DELAY_MS)
  {
    return;
  }

  if (!start_logged_)
  {
    mqttLog("MQTT starting WiFi");
    start_logged_ = true;
  }

  ensureWifi(now);
  ensureMqtt(now);

  if (mqtt_client_.connected())
  {
    mqtt_client_.loop();
  }
}

bool MqttTrafficGate::isEnabled() const
{
  return enabled_;
}

bool MqttTrafficGate::isConnected()
{
  return enabled_ && WiFi.status() == WL_CONNECTED && mqtt_client_.connected();
}

bool MqttTrafficGate::hasFreshSpat() const
{
  return last_spat_ms_ != 0 && millis() - last_spat_ms_ <= MQTT_SPAT_STALE_MS;
}

float MqttTrafficGate::speedMultiplier() const
{
  if (!enabled_)
  {
    return 1.0f;
  }

  if (!hasFreshSpat())
  {
    return 0.0f;
  }

  if (movement_state_ == TrafficMovementState::Allowed)
  {
    return 1.0f;
  }

  if (movement_state_ == TrafficMovementState::Clearance)
  {
    return 0.5f;
  }

  return 0.0f;
}

const char *MqttTrafficGate::movementStateName() const
{
  if (enabled_ && !hasFreshSpat())
  {
    return "stale-red";
  }

  return stateName(movement_state_);
}

int MqttTrafficGate::signalGroup() const
{
  return signal_group_;
}

int MqttTrafficGate::timeRemaining() const
{
  return time_remaining_;
}

void MqttTrafficGate::setSignalGroup(int signal_group)
{
  if (signal_group < 1 || signal_group == signal_group_)
  {
    return;
  }

  signal_group_ = signal_group;
  last_spat_ms_ = 0;
  movement_state_ = TrafficMovementState::Unknown;
  time_remaining_ = -1;
}

void MqttTrafficGate::ensureWifi(unsigned long now_ms)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    return;
  }

  if (wifi_started_ && now_ms - last_wifi_attempt_ms_ < MQTT_WIFI_RECONNECT_INTERVAL_MS)
  {
    return;
  }

  bool first_attempt = !wifi_started_;
  wifi_started_ = true;
  last_wifi_attempt_ms_ = now_ms;
  if (first_attempt)
  {
    mqttLog("MQTT WiFi begin");
    WiFi.begin(MQTT_WIFI_SSID, MQTT_WIFI_PASSWORD);
  }
  else
  {
    mqttLog("MQTT WiFi reconnect");
    WiFi.reconnect();
  }
}

void MqttTrafficGate::ensureMqtt(unsigned long now_ms)
{
  if (WiFi.status() != WL_CONNECTED || mqtt_client_.connected())
  {
    return;
  }

  if (now_ms - last_mqtt_attempt_ms_ < MQTT_RECONNECT_INTERVAL_MS)
  {
    return;
  }

  last_mqtt_attempt_ms_ = now_ms;
  mqttLog("MQTT connecting broker");
  if (mqtt_client_.connect(client_id_))
  {
    mqttLog("MQTT broker connected");
    subscribeTopics();
  }
}

void MqttTrafficGate::subscribeTopics()
{
  mqtt_client_.subscribe(MQTT_SPAT_TOPIC);
  mqtt_client_.subscribe(MQTT_MAP_TOPIC);
  if (!subscriptions_logged_)
  {
    mqttLog("MQTT subscribed to SPaT/MAP");
    subscriptions_logged_ = true;
  }
}

void MqttTrafficGate::handleMessage(char *topic, const uint8_t *payload, unsigned int length)
{
  char message[MQTT_MESSAGE_BUFFER_BYTES + 1];
  unsigned int copy_length = length;
  if (copy_length > MQTT_MESSAGE_BUFFER_BYTES)
  {
    copy_length = MQTT_MESSAGE_BUFFER_BYTES;
  }

  memcpy(message, payload, copy_length);
  message[copy_length] = '\0';

  if (strcmp(topic, MQTT_SPAT_TOPIC) == 0)
  {
    handleSpat(message);
  }
  else if (strcmp(topic, MQTT_MAP_TOPIC) == 0)
  {
    handleMap(message);
  }
}

void MqttTrafficGate::handleSpat(const char *payload)
{
  DynamicJsonDocument document(MQTT_MESSAGE_BUFFER_BYTES * 2);
  if (deserializeJson(document, payload))
  {
    return;
  }

  TrafficMovementState parsed_state = TrafficMovementState::Unknown;
  int parsed_time_remaining = -1;
  if (parseSpat(document.as<JsonVariantConst>(), signal_group_, parsed_state, parsed_time_remaining))
  {
    movement_state_ = parsed_state;
    time_remaining_ = parsed_time_remaining;
    last_spat_ms_ = millis();
#if MQTT_SERIAL_DEBUG
    Serial.printf("MQTT SPaT group=%d state=%s t=%d mul=%.1f\n", signal_group_, movementStateName(), time_remaining_, speedMultiplier());
#endif
  }
}

void MqttTrafficGate::handleMap(const char *payload)
{
  DynamicJsonDocument document(MQTT_MESSAGE_BUFFER_BYTES * 2);
  if (deserializeJson(document, payload))
  {
    return;
  }

  int mapped_signal_group = 0;
  if (parseMap(document.as<JsonVariantConst>(), mapped_signal_group))
  {
    setSignalGroup(mapped_signal_group);
#if MQTT_SERIAL_DEBUG
    Serial.printf("MQTT MAP signalGroup=%d\n", signal_group_);
#endif
  }
}
