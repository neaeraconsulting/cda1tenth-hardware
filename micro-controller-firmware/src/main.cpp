#include <Arduino.h>
#include <BLE2902.h>
#include <BLEAdvertising.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <stdio.h>
#include <string.h>

#include "car.h"
#include "sensor_manager.h"

// Pin definitions
#define LED_PIN 37

// Default parameter values
#define DEFAULT_ENCODER_OFFSET 187.5f
#define DEFAULT_WHEELBASE 0.185f
#define DEFAULT_TRACK_WIDTH 0.15f

// Control constants
#define MIN_VELOCITY_THRESHOLD 0.01f
#define MAX_STEERING_ANGLE_DEG 30.0f
#define WHEEL_RADIUS_M 0.0325f
#define MAX_RPM 300.0f
#define MAX_LINEAR_SPEED_MPS 1.0f
#define MAX_ANGULAR_SPEED_RADPS 3.0f
#define COMMAND_TIMEOUT_MS 2000
#define CONNECTED_LED_BLINK_MS 250
#define BLE_DEBUG_RESPONSES 0
#define INVERT_JOYSTICK_X 1
#define INVERT_JOYSTICK_X_WHEN_REVERSING 1

const char *serviceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char *rxCharacteristicUuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
const char *txCharacteristicUuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
const char *bleDeviceName = "CDA1Tenth";

const uint8_t packetHeader1 = 0xFA;
const uint8_t packetHeader2 = 0xFB;
const uint8_t packetFooter = 0xFE;
const uint8_t axisMax = 100;
const size_t packetLength = 10;
const uint8_t zeroSteerButton = 0x01;

// Global objects
Car car(CS_RIGHT, CS_LEFT, CS_STEER);
SensorManager sensor_manager;
Preferences preferences;
BLECharacteristic *txCharacteristic = nullptr;

// Command state
float command_linear_x = 0.0f;
float command_angular_z = 0.0f;
float command_speed_rpm = 0.0f;
uint8_t last_buttons = 0;
unsigned long last_command_ms = 0;
bool ble_device_connected = false;

// Runtime parameters
float encoder_offset = DEFAULT_ENCODER_OFFSET;
bool preferences_ready = false;
float max_steering_angle = MAX_STEERING_ANGLE_DEG;
float max_rpm = MAX_RPM;
float wheel_radius = WHEEL_RADIUS_M;

void flashLED(int n_times);
void updateStatusLed();
void moveBase();
void setupBle();
void sendBleResponse(const char *message);
void handleJoystickPacket(const std::string &packet);
void handleBleTextCommand(const std::string &packet);
int decodeAxisValue(uint8_t sign, uint8_t value);
void printStatus();
void setSteeringOffset(float offset, bool save);
void zeroSteering();
void stopCommand();

class ServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *server) override
  {
    (void)server;
    ble_device_connected = true;
  }

  void onDisconnect(BLEServer *server) override
  {
    ble_device_connected = false;
    stopCommand();
    server->startAdvertising();
  }
};

class JoystickCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *characteristic) override
  {
    handleJoystickPacket(characteristic->getValue());
  }
};

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  SPI.begin();
  SPI.setFrequency(1000000);

  if (!sensor_manager.initialize())
  {
    flashLED(3);
  }

  car.begin();
  setupBle();

  delay(2000);
  flashLED(2);

  preferences_ready = preferences.begin("cda1", false);
  encoder_offset = preferences_ready ? preferences.getFloat("steer_ofs", DEFAULT_ENCODER_OFFSET) : DEFAULT_ENCODER_OFFSET;
  setSteeringOffset(encoder_offset, false);

  car.wheelbase = DEFAULT_WHEELBASE;
  car.trackWidth = DEFAULT_TRACK_WIDTH;
}

void loop()
{
  static unsigned long last_control_ms = 0;
#if BLE_DEBUG_RESPONSES
  static unsigned long last_status_ms = 0;
#endif

  updateStatusLed();

  unsigned long now = millis();
  if (last_command_ms != 0 && now - last_command_ms > COMMAND_TIMEOUT_MS)
  {
    stopCommand();
  }

  if (now - last_control_ms >= 20)
  {
    sensor_manager.update();
    moveBase();
    last_control_ms = now;
  }

  car.updateControlLoops();

#if BLE_DEBUG_RESPONSES
  if (now - last_status_ms >= 1000)
  {
    printStatus();
    last_status_ms = now;
  }
#endif
}

void setupBle()
{
  BLEDevice::init(bleDeviceName);
  BLEDevice::setMTU(185);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(serviceUuid);

  txCharacteristic = service->createCharacteristic(
      txCharacteristicUuid,
      BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *rxCharacteristic = service->createCharacteristic(
      rxCharacteristicUuid,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxCharacteristic->setCallbacks(new JoystickCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  BLEAdvertisementData advertisementData;
  advertisementData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  advertisementData.setCompleteServices(BLEUUID(serviceUuid));

  BLEAdvertisementData scanResponseData;
  scanResponseData.setName(bleDeviceName);

  advertising->setAdvertisementData(advertisementData);
  advertising->setScanResponseData(scanResponseData);
  advertising->setScanResponse(true);
  advertising->start();
}

void sendBleResponse(const char *message)
{
  if (txCharacteristic == nullptr || !ble_device_connected)
  {
    return;
  }

  txCharacteristic->setValue(message);
  txCharacteristic->notify();
}

void handleJoystickPacket(const std::string &packet)
{
  if (packet.length() != packetLength)
  {
    handleBleTextCommand(packet);
    return;
  }

  uint8_t header1 = packet[0];
  uint8_t header2 = packet[1];
  uint8_t xSign = packet[2];
  uint8_t xValue = packet[3];
  uint8_t ySign = packet[4];
  uint8_t yValue = packet[5];
  uint8_t buttons = packet[6];
  uint8_t reserved = packet[7];
  uint8_t receivedChecksum = packet[8];
  uint8_t footer = packet[9];

  if (header1 != packetHeader1 || header2 != packetHeader2 || footer != packetFooter)
  {
#if BLE_DEBUG_RESPONSES
    sendBleResponse("ERR packet markers");
#endif
    return;
  }

  uint8_t checksum = 0;
  for (size_t i = 0; i < 8; i++)
  {
    checksum += static_cast<uint8_t>(packet[i]);
  }

  bool checksum_ok = receivedChecksum == checksum;
#if BLE_DEBUG_RESPONSES
  if (!checksum_ok)
  {
    char message[96];
    snprintf(message, sizeof(message), "ERR checksum got %02X expected %02X", receivedChecksum, checksum);
    sendBleResponse(message);
  }
#endif

  int x = decodeAxisValue(xSign, xValue);
  int y = decodeAxisValue(ySign, yValue);

  command_linear_x = (static_cast<float>(y) / axisMax) * MAX_LINEAR_SPEED_MPS;
#if INVERT_JOYSTICK_X
  x = -x;
#endif
#if INVERT_JOYSTICK_X_WHEN_REVERSING
  if (y < 0)
  {
    x = -x;
  }
#endif
  command_angular_z = (static_cast<float>(x) / axisMax) * MAX_ANGULAR_SPEED_RADPS;
  command_speed_rpm = (command_linear_x / wheel_radius) * (60.0f / (2.0f * M_PI));
  last_command_ms = millis();

  if ((buttons & zeroSteerButton) != 0 && (last_buttons & zeroSteerButton) == 0)
  {
    zeroSteering();
  }

  last_buttons = buttons;

#if BLE_DEBUG_RESPONSES
  char message[96];
  snprintf(message, sizeof(message), "%s X=%d Y=%d BTN=%02X RSV=%02X", checksum_ok ? "OK" : "WARN", x, y, buttons, reserved);
  sendBleResponse(message);
#else
  (void)checksum_ok;
  (void)reserved;
#endif
}

int decodeAxisValue(uint8_t sign, uint8_t value)
{
  int axis = 0;

  // UniControl has shown both sign+magnitude-ish values and signed-byte values
  // like FF FF for -1, so decode the second byte as signed when it is out of
  // the joystick's normal 0..100 magnitude range.
  if (value <= axisMax)
  {
    axis = value;
    if (sign == 0xFF || sign == 0x01)
    {
      axis = -axis;
    }
  }
  else
  {
    axis = static_cast<int8_t>(value);
  }

  if (axis > axisMax)
  {
    axis = axisMax;
  }
  else if (axis < -axisMax)
  {
    axis = -axisMax;
  }

  return axis;
}

void handleBleTextCommand(const std::string &packet)
{
  char line[64];
  char command[16] = {0};
  float first_value = 0.0f;
  float second_value = 0.0f;
  size_t length = packet.length();

  if (length >= sizeof(line))
  {
    length = sizeof(line) - 1;
  }

  memcpy(line, packet.data(), length);
  line[length] = '\0';

  int parsed = sscanf(line, "%15s %f %f", command, &first_value, &second_value);
  if (parsed <= 0)
  {
    return;
  }

  if (strcmp(command, "v") == 0 && parsed >= 3)
  {
    command_linear_x = first_value;
    command_angular_z = second_value;
    command_speed_rpm = (command_linear_x / wheel_radius) * (60.0f / (2.0f * M_PI));
    last_command_ms = millis();

    char message[96];
    snprintf(message, sizeof(message), "OK v=%.2f w=%.2f rpm=%.1f", command_linear_x, command_angular_z, command_speed_rpm);
    sendBleResponse(message);
  }
  else if (strcmp(command, "stop") == 0)
  {
    stopCommand();
    sendBleResponse("OK stopped");
  }
  else if (strcmp(command, "zero_steer") == 0)
  {
    zeroSteering();
  }
  else if (strcmp(command, "status") == 0)
  {
    printStatus();
  }
  else
  {
    char message[96];
    snprintf(message, sizeof(message), "ERR unknown/len %u", static_cast<unsigned int>(packet.length()));
    sendBleResponse(message);
  }
}

void printStatus()
{
  char message[96];
  snprintf(
      message,
      sizeof(message),
      "S v=%.2f w=%.2f rpm=%.1f steer=%.1f actual=%.1f ofs=%.1f",
      command_linear_x,
      command_angular_z,
      command_speed_rpm,
      car.steeringAngle,
      car.getActualSteeringAngle(),
      encoder_offset);
  sendBleResponse(message);
}

void setSteeringOffset(float offset, bool save)
{
  encoder_offset = offset;
  car.steeringMotor.setEncoderOffset(encoder_offset);

  if (save && preferences_ready)
  {
    preferences.putFloat("steer_ofs", encoder_offset);
  }
}

void zeroSteering()
{
  float raw_angle = car.steeringMotor.getSteeringAngle();
  stopCommand();
  setSteeringOffset(raw_angle, true);
  car.setSteeringAngle(0.0f);
  sendBleResponse("OK steering zeroed");
}

void stopCommand()
{
  command_linear_x = 0.0f;
  command_angular_z = 0.0f;
  command_speed_rpm = 0.0f;
  last_command_ms = 0;
  last_buttons = 0;
  car.emergencyStop();
}

void moveBase()
{
  float linear_x = command_linear_x;
  float angular_z = command_angular_z;
  float steering_angle = 0.0f;

  if (fabs(linear_x) > MIN_VELOCITY_THRESHOLD && fabs(angular_z) > MIN_VELOCITY_THRESHOLD)
  {
    float turning_radius = linear_x / angular_z;
    steering_angle = atanf(car.wheelbase / turning_radius) * (180.0f / M_PI);
  }

  if (steering_angle > max_steering_angle)
    steering_angle = max_steering_angle;
  if (steering_angle < -max_steering_angle)
    steering_angle = -max_steering_angle;

  float speed_rpm = command_speed_rpm;

  if (speed_rpm > max_rpm)
    speed_rpm = max_rpm;
  if (speed_rpm < -max_rpm)
    speed_rpm = -max_rpm;

  car.setSteeringAngle(steering_angle);
  car.setSpeed(speed_rpm, car.wheelbase, car.trackWidth);
  car.updateControlLoops();
}

void flashLED(int n_times)
{
  for (int i = 0; i < n_times; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  delay(1000);
}

void updateStatusLed()
{
  static unsigned long last_toggle_ms = 0;
  static bool led_on = false;

  if (!ble_device_connected)
  {
    led_on = true;
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  unsigned long now = millis();
  if (now - last_toggle_ms >= CONNECTED_LED_BLINK_MS)
  {
    led_on = !led_on;
    digitalWrite(LED_PIN, led_on ? HIGH : LOW);
    last_toggle_ms = now;
  }
}
