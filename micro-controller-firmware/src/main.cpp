#include <Arduino.h>
#include "USB.h"
#include "USBCDC.h"
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

// Global objects
Car car(CS_RIGHT, CS_LEFT, CS_STEER);
SensorManager sensor_manager;
USBCDC USBSerial;
Preferences preferences;

// Command state
float command_linear_x = 0.0f;
float command_angular_z = 0.0f;
float command_speed_rpm = 0.0f;
float manual_steering_angle = 0.0f;
bool manual_steering_active = false;

// Runtime parameters
float encoder_offset = DEFAULT_ENCODER_OFFSET;
bool preferences_ready = false;
float max_steering_angle = MAX_STEERING_ANGLE_DEG;
float max_rpm = MAX_RPM;
float wheel_radius = WHEEL_RADIUS_M;

void flashLED(int n_times);
void moveBase();
void handleUsbCommands();
void handleUsbCommand(char *line);
void printUsbHelp();
void printStatus();
void setSteeringOffset(float offset, bool save);

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
  USB.begin();
  USBSerial.begin(115200);

  while (!USBSerial)
  {
    delay(10);
    flashLED(1);
  }

  delay(2000);
  flashLED(2);

  preferences_ready = preferences.begin("cda1", false);
  encoder_offset = preferences_ready ? preferences.getFloat("steer_ofs", DEFAULT_ENCODER_OFFSET) : DEFAULT_ENCODER_OFFSET;
  setSteeringOffset(encoder_offset, false);

  car.wheelbase = DEFAULT_WHEELBASE;
  car.trackWidth = DEFAULT_TRACK_WIDTH;

  printUsbHelp();
}

void loop()
{
  static unsigned long last_control_ms = 0;
  static unsigned long last_status_ms = 0;

  handleUsbCommands();

  unsigned long now = millis();
  if (now - last_control_ms >= 20)
  {
    sensor_manager.update();
    moveBase();
    last_control_ms = now;
  }

  car.updateControlLoops();

  if (now - last_status_ms >= 1000)
  {
    printStatus();
    last_status_ms = now;
  }
}

void handleUsbCommands()
{
  static char line[64];
  static size_t length = 0;

  while (USBSerial.available() > 0)
  {
    char c = (char)USBSerial.read();
    if (c == '\r')
    {
      continue;
    }

    if (c == '\n')
    {
      line[length] = '\0';
      handleUsbCommand(line);
      length = 0;
      continue;
    }

    if (length < sizeof(line) - 1)
    {
      line[length++] = c;
    }
  }
}

void handleUsbCommand(char *line)
{
  char command[16] = {0};
  float first_value = 0.0f;
  float second_value = 0.0f;
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
    manual_steering_active = false;
    USBSerial.printf("cmd velocity: linear=%.3f m/s angular=%.3f rad/s\r\n", command_linear_x, command_angular_z);
  }
  else if (strcmp(command, "speed") == 0 && parsed >= 2)
  {
    command_speed_rpm = first_value;
    command_linear_x = command_speed_rpm * (2.0f * M_PI * wheel_radius) / 60.0f;
    USBSerial.printf("cmd speed: %.3f rpm\r\n", first_value);
  }
  else if (strcmp(command, "steer") == 0 && parsed >= 2)
  {
    manual_steering_angle = first_value;
    manual_steering_active = true;
    car.setSteeringAngle(manual_steering_angle);
    USBSerial.printf("cmd steering: %.3f deg\r\n", first_value);
  }
  else if (strcmp(command, "stop") == 0)
  {
    command_linear_x = 0.0f;
    command_angular_z = 0.0f;
    command_speed_rpm = 0.0f;
    manual_steering_active = false;
    car.emergencyStop();
    USBSerial.println("stopped");
  }
  else if (strcmp(command, "zero_steer") == 0)
  {
    float raw_angle = car.steeringMotor.getSteeringAngle();
    command_linear_x = 0.0f;
    command_angular_z = 0.0f;
    command_speed_rpm = 0.0f;
    manual_steering_angle = 0.0f;
    manual_steering_active = true;
    setSteeringOffset(raw_angle, true);
    car.setSteeringAngle(0.0f);
    USBSerial.printf("steering zeroed: offset=%.3f deg\r\n", encoder_offset);
  }
  else if (strcmp(command, "offset") == 0 && parsed >= 2)
  {
    setSteeringOffset(first_value, true);
    USBSerial.printf("steering offset set: offset=%.3f deg\r\n", encoder_offset);
  }
  else if (strcmp(command, "status") == 0)
  {
    printStatus();
  }
  else if (strcmp(command, "help") == 0)
  {
    printUsbHelp();
  }
  else
  {
    USBSerial.println("unknown command");
    printUsbHelp();
  }
}

void printUsbHelp()
{
  USBSerial.println();
  USBSerial.println("CDA1Tenth USB bring-up mode.");
  USBSerial.println("Commands:");
  USBSerial.println("  v <linear_mps> <angular_radps>");
  USBSerial.println("  speed <rpm>");
  USBSerial.println("  steer <deg>");
  USBSerial.println("  zero_steer");
  USBSerial.println("  offset <deg>");
  USBSerial.println("  stop");
  USBSerial.println("  status");
  USBSerial.println("  help");
  USBSerial.println();
}

void printStatus()
{
  SensorData sensor_data = sensor_manager.getLatestData();
  USBSerial.printf(
      "status ms=%lu cmd_v=%.3f cmd_w=%.3f cmd_rpm=%.3f speed=%.3f steer=%.3f actual_steer=%.3f offset=%.3f rpm_r=%.3f rpm_l=%.3f accel=(%.3f,%.3f,%.3f) gyro=(%.3f,%.3f,%.3f)\r\n",
      millis(),
      command_linear_x,
      command_angular_z,
      command_speed_rpm,
      car.speed,
      car.steeringAngle,
      car.getActualSteeringAngle(),
      encoder_offset,
      car.getRightMotorRPM(),
      car.getLeftMotorRPM(),
      sensor_data.accel_x,
      sensor_data.accel_y,
      sensor_data.accel_z,
      sensor_data.gyro_x,
      sensor_data.gyro_y,
      sensor_data.gyro_z);
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

void moveBase()
{
  float linear_x = command_linear_x;
  float angular_z = command_angular_z;
  float steering_angle = 0.0f;

  if (manual_steering_active)
  {
    steering_angle = manual_steering_angle;
  }
  else if (fabs(linear_x) > MIN_VELOCITY_THRESHOLD && fabs(angular_z) > MIN_VELOCITY_THRESHOLD)
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
