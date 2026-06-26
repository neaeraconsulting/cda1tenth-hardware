#ifndef VEHICLE_LIGHTS_H
#define VEHICLE_LIGHTS_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

struct RgbColor
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

class FrontLights
{
public:
  enum class SignalState
  {
    Off,
    Left,
    Right,
    Hazard
  };

  static const uint8_t LightCount = 7;
  static const uint8_t LeftInnerSignalIndex = 5;
  static const uint8_t LeftOuterSignalIndex = 6;
  static const uint8_t RightOuterSignalIndex = 0;
  static const uint8_t RightInnerSignalIndex = 1;
  static const uint8_t HeadlightLeftIndex = 2;
  static const uint8_t HeadlightCenterIndex = 3;
  static const uint8_t HeadlightRightIndex = 4;

  FrontLights(uint8_t pin);

  void begin();
  void update(unsigned long now_ms);

  void setWaitingForConnection(bool waiting);
  void playConnectionBlink();
  void setSignalState(SignalState state);
  SignalState getSignalState() const;

  void setHeadlights(bool enabled);
  bool getHeadlights() const;
  void setHeadlightColor(RgbColor color);
  void setSignalColor(RgbColor color);
  void setPixel(uint8_t index, RgbColor color);
  void clearPixel(uint8_t index);
  void clearAll();

private:
  static const uint16_t SignalBlinkMs = 350;
  static const uint16_t ConnectionAnimationMs = 180;
  static const uint16_t ConnectedBlinkMs = 220;

  Adafruit_NeoPixel strip_;
  RgbColor base_pixels_[LightCount];
  RgbColor headlight_color_ = {180, 180, 160};
  RgbColor signal_color_ = {255, 90, 0};
  SignalState signal_state_ = SignalState::Off;
  unsigned long last_signal_toggle_ms_ = 0;
  bool signal_on_ = false;
  bool headlights_enabled_ = false;
  bool waiting_for_connection_ = false;
  uint8_t connection_phase_ = 0;
  unsigned long last_connection_step_ms_ = 0;
  bool connected_blink_active_ = false;
  uint8_t connected_blink_phase_ = 0;
  unsigned long last_connected_blink_step_ms_ = 0;
  bool initialized_ = false;

  void render();
  void advanceConnectionAnimation(unsigned long now_ms);
  void renderConnectionAnimation();
  void advanceConnectedBlink(unsigned long now_ms);
  void renderConnectedBlink();
  void setOutputPixel(uint8_t index, RgbColor color);
  RgbColor scaleColor(RgbColor color, uint8_t numerator, uint8_t denominator) const;
  uint32_t toStripColor(RgbColor color) const;
};

class RearLights
{
public:
  static const uint8_t LightCount = 7;
  static const uint8_t RightOuterSignalIndex = 0;
  static const uint8_t RightInnerSignalIndex = 1;
  static const uint8_t BrakeLeftIndex = 2;
  static const uint8_t BrakeCenterIndex = 3;
  static const uint8_t BrakeRightIndex = 4;
  static const uint8_t LeftInnerSignalIndex = 5;
  static const uint8_t LeftOuterSignalIndex = 6;

  RearLights(uint8_t pin);

  void begin();
  void update(unsigned long now_ms);

  void setWaitingForConnection(bool waiting);
  void playConnectionBlink();
  void setSignalState(FrontLights::SignalState state);
  void setBrakeLights(bool enabled);
  void setReverse(bool enabled);
  void setBrakeColor(RgbColor color);
  void setSignalColor(RgbColor color);
  void setReverseColor(RgbColor color);

private:
  static const uint16_t SignalBlinkMs = 350;
  static const uint16_t ConnectionAnimationMs = 180;
  static const uint16_t ConnectedBlinkMs = 220;

  Adafruit_NeoPixel strip_;
  RgbColor brake_color_ = {255, 0, 0};
  RgbColor signal_color_ = {255, 90, 0};
  RgbColor reverse_color_ = {180, 180, 180};
  FrontLights::SignalState signal_state_ = FrontLights::SignalState::Off;
  unsigned long last_signal_toggle_ms_ = 0;
  bool signal_on_ = false;
  bool brake_lights_enabled_ = false;
  bool reverse_enabled_ = false;
  bool waiting_for_connection_ = false;
  uint8_t connection_phase_ = 0;
  unsigned long last_connection_step_ms_ = 0;
  bool connected_blink_active_ = false;
  uint8_t connected_blink_phase_ = 0;
  unsigned long last_connected_blink_step_ms_ = 0;
  bool initialized_ = false;

  void render();
  void advanceConnectionAnimation(unsigned long now_ms);
  void renderConnectionAnimation();
  void advanceConnectedBlink(unsigned long now_ms);
  void renderConnectedBlink();
  void setOutputPixel(uint8_t index, RgbColor color);
  RgbColor scaleColor(RgbColor color, uint8_t numerator, uint8_t denominator) const;
  uint32_t toStripColor(RgbColor color) const;
};

#endif // VEHICLE_LIGHTS_H
