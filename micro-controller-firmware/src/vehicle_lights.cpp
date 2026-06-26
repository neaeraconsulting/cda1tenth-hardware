#include "vehicle_lights.h"

static const RgbColor OffColor = {0, 0, 0};
static const RgbColor BluetoothBlue = {0, 45, 127};
static const uint8_t SignalBrightnessNumerator = 2;
static const uint8_t SignalBrightnessDenominator = 3;

FrontLights::FrontLights(uint8_t pin)
    : strip_(LightCount, pin, NEO_GRB + NEO_KHZ800)
{
  clearAll();
}

void FrontLights::begin()
{
  strip_.begin();
  initialized_ = true;
  strip_.clear();
  render();
}

void FrontLights::update(unsigned long now_ms)
{
  if (connected_blink_active_)
  {
    advanceConnectedBlink(now_ms);
    return;
  }

  if (waiting_for_connection_)
  {
    advanceConnectionAnimation(now_ms);
    return;
  }

  if (signal_state_ != SignalState::Off && now_ms - last_signal_toggle_ms_ >= SignalBlinkMs)
  {
    signal_on_ = !signal_on_;
    last_signal_toggle_ms_ = now_ms;
    render();
  }
}

void FrontLights::setWaitingForConnection(bool waiting)
{
  if (waiting_for_connection_ == waiting)
  {
    return;
  }

  waiting_for_connection_ = waiting;
  if (waiting_for_connection_)
  {
    connected_blink_active_ = false;
  }
  connection_phase_ = 0;
  last_connection_step_ms_ = millis();
  render();
}

void FrontLights::playConnectionBlink()
{
  if (waiting_for_connection_)
  {
    return;
  }

  connected_blink_active_ = true;
  connected_blink_phase_ = 0;
  last_connected_blink_step_ms_ = millis();
  render();
}

void FrontLights::setSignalState(SignalState state)
{
  signal_state_ = state;
  signal_on_ = state != SignalState::Off;
  last_signal_toggle_ms_ = millis();
  render();
}

FrontLights::SignalState FrontLights::getSignalState() const
{
  return signal_state_;
}

void FrontLights::setHeadlights(bool enabled)
{
  headlights_enabled_ = enabled;
  render();
}

bool FrontLights::getHeadlights() const
{
  return headlights_enabled_;
}

void FrontLights::setHeadlightColor(RgbColor color)
{
  headlight_color_ = color;
  render();
}

void FrontLights::setSignalColor(RgbColor color)
{
  signal_color_ = color;
  render();
}

void FrontLights::setPixel(uint8_t index, RgbColor color)
{
  if (index >= LightCount)
  {
    return;
  }

  base_pixels_[index] = color;
  render();
}

void FrontLights::clearPixel(uint8_t index)
{
  setPixel(index, OffColor);
}

void FrontLights::clearAll()
{
  for (uint8_t i = 0; i < LightCount; i++)
  {
    base_pixels_[i] = OffColor;
  }

  render();
}

void FrontLights::render()
{
  if (!initialized_)
  {
    return;
  }

  if (waiting_for_connection_)
  {
    renderConnectionAnimation();
    strip_.show();
    return;
  }

  if (connected_blink_active_)
  {
    renderConnectedBlink();
    strip_.show();
    return;
  }

  for (uint8_t i = 0; i < LightCount; i++)
  {
    setOutputPixel(i, base_pixels_[i]);
  }

  if (headlights_enabled_)
  {
    setOutputPixel(HeadlightLeftIndex, headlight_color_);
    setOutputPixel(HeadlightCenterIndex, headlight_color_);
    setOutputPixel(HeadlightRightIndex, headlight_color_);
  }

  bool left_signal_active = signal_on_ && (signal_state_ == SignalState::Left || signal_state_ == SignalState::Hazard);
  bool right_signal_active = signal_on_ && (signal_state_ == SignalState::Right || signal_state_ == SignalState::Hazard);
  RgbColor dim_signal_color = scaleColor(signal_color_, SignalBrightnessNumerator, SignalBrightnessDenominator);

  setOutputPixel(LeftOuterSignalIndex, left_signal_active ? dim_signal_color : OffColor);
  setOutputPixel(LeftInnerSignalIndex, left_signal_active ? dim_signal_color : OffColor);
  setOutputPixel(RightInnerSignalIndex, right_signal_active ? dim_signal_color : OffColor);
  setOutputPixel(RightOuterSignalIndex, right_signal_active ? dim_signal_color : OffColor);

  strip_.show();
}

void FrontLights::advanceConnectionAnimation(unsigned long now_ms)
{
  if (now_ms - last_connection_step_ms_ < ConnectionAnimationMs)
  {
    return;
  }

  connection_phase_++;
  if (connection_phase_ >= LightCount * 2)
  {
    connection_phase_ = 0;
  }

  last_connection_step_ms_ = now_ms;
  render();
}

void FrontLights::renderConnectionAnimation()
{
  for (uint8_t i = 0; i < LightCount; i++)
  {
    setOutputPixel(i, OffColor);
  }

  int first_lit = LightCount - 1 - connection_phase_;
  int last_lit = LightCount - 1;
  if (connection_phase_ >= LightCount)
  {
    first_lit = 0;
    last_lit = (LightCount * 2) - connection_phase_ - 2;
  }

  for (int i = first_lit; i <= last_lit && i < LightCount; i++)
  {
    if (i >= 0)
    {
      setOutputPixel(static_cast<uint8_t>(i), BluetoothBlue);
    }
  }
}

void FrontLights::advanceConnectedBlink(unsigned long now_ms)
{
  if (now_ms - last_connected_blink_step_ms_ < ConnectedBlinkMs)
  {
    return;
  }

  connected_blink_phase_++;
  last_connected_blink_step_ms_ = now_ms;

  if (connected_blink_phase_ >= 4)
  {
    connected_blink_active_ = false;
  }

  render();
}

void FrontLights::renderConnectedBlink()
{
  bool blink_on = (connected_blink_phase_ % 2) == 0;

  for (uint8_t i = 0; i < LightCount; i++)
  {
    setOutputPixel(i, blink_on ? BluetoothBlue : OffColor);
  }
}

void FrontLights::setOutputPixel(uint8_t index, RgbColor color)
{
  if (index >= LightCount)
  {
    return;
  }

  strip_.setPixelColor(index, toStripColor(color));
}

RgbColor FrontLights::scaleColor(RgbColor color, uint8_t numerator, uint8_t denominator) const
{
  if (denominator == 0)
  {
    return color;
  }

  return {
      static_cast<uint8_t>((static_cast<uint16_t>(color.red) * numerator) / denominator),
      static_cast<uint8_t>((static_cast<uint16_t>(color.green) * numerator) / denominator),
      static_cast<uint8_t>((static_cast<uint16_t>(color.blue) * numerator) / denominator)};
}

uint32_t FrontLights::toStripColor(RgbColor color) const
{
  return strip_.Color(color.red, color.green, color.blue);
}

RearLights::RearLights(uint8_t pin)
    : strip_(LightCount, pin, NEO_GRB + NEO_KHZ800)
{
}

void RearLights::begin()
{
  strip_.begin();
  initialized_ = true;
  strip_.clear();
  render();
}

void RearLights::update(unsigned long now_ms)
{
  if (connected_blink_active_)
  {
    advanceConnectedBlink(now_ms);
    return;
  }

  if (waiting_for_connection_)
  {
    advanceConnectionAnimation(now_ms);
    return;
  }

  bool should_render = false;

  if (signal_state_ != FrontLights::SignalState::Off && now_ms - last_signal_toggle_ms_ >= SignalBlinkMs)
  {
    signal_on_ = !signal_on_;
    last_signal_toggle_ms_ = now_ms;
    should_render = true;
  }

  if (should_render)
  {
    render();
  }
}

void RearLights::setWaitingForConnection(bool waiting)
{
  if (waiting_for_connection_ == waiting)
  {
    return;
  }

  waiting_for_connection_ = waiting;
  if (waiting_for_connection_)
  {
    connected_blink_active_ = false;
  }
  connection_phase_ = 0;
  last_connection_step_ms_ = millis();
  render();
}

void RearLights::playConnectionBlink()
{
  if (waiting_for_connection_)
  {
    return;
  }

  connected_blink_active_ = true;
  connected_blink_phase_ = 0;
  last_connected_blink_step_ms_ = millis();
  render();
}

void RearLights::setSignalState(FrontLights::SignalState state)
{
  signal_state_ = state;
  signal_on_ = state != FrontLights::SignalState::Off;
  last_signal_toggle_ms_ = millis();
  render();
}

void RearLights::setBrakeLights(bool enabled)
{
  if (brake_lights_enabled_ == enabled)
  {
    return;
  }

  brake_lights_enabled_ = enabled;
  render();
}

void RearLights::setReverse(bool enabled)
{
  if (reverse_enabled_ == enabled)
  {
    return;
  }

  reverse_enabled_ = enabled;
  render();
}

void RearLights::setBrakeColor(RgbColor color)
{
  brake_color_ = color;
  render();
}

void RearLights::setSignalColor(RgbColor color)
{
  signal_color_ = color;
  render();
}

void RearLights::setReverseColor(RgbColor color)
{
  reverse_color_ = color;
  render();
}

void RearLights::render()
{
  if (!initialized_)
  {
    return;
  }

  if (waiting_for_connection_)
  {
    renderConnectionAnimation();
    strip_.show();
    return;
  }

  if (connected_blink_active_)
  {
    renderConnectedBlink();
    strip_.show();
    return;
  }

  for (uint8_t i = 0; i < LightCount; i++)
  {
    setOutputPixel(i, OffColor);
  }

  if (brake_lights_enabled_)
  {
    setOutputPixel(BrakeLeftIndex, brake_color_);
    setOutputPixel(BrakeCenterIndex, brake_color_);
    setOutputPixel(BrakeRightIndex, brake_color_);
  }

  bool left_signal_active = signal_on_ && (signal_state_ == FrontLights::SignalState::Left || signal_state_ == FrontLights::SignalState::Hazard);
  bool right_signal_active = signal_on_ && (signal_state_ == FrontLights::SignalState::Right || signal_state_ == FrontLights::SignalState::Hazard);
  RgbColor dim_signal_color = scaleColor(signal_color_, SignalBrightnessNumerator, SignalBrightnessDenominator);

  setOutputPixel(LeftOuterSignalIndex, left_signal_active ? dim_signal_color : OffColor);
  setOutputPixel(LeftInnerSignalIndex, left_signal_active ? dim_signal_color : OffColor);
  setOutputPixel(RightInnerSignalIndex, right_signal_active ? dim_signal_color : OffColor);
  setOutputPixel(RightOuterSignalIndex, right_signal_active ? dim_signal_color : OffColor);

  if (reverse_enabled_)
  {
    setOutputPixel(LeftOuterSignalIndex, reverse_color_);
    setOutputPixel(RightOuterSignalIndex, reverse_color_);
  }

  strip_.show();
}

void RearLights::advanceConnectionAnimation(unsigned long now_ms)
{
  if (now_ms - last_connection_step_ms_ < ConnectionAnimationMs)
  {
    return;
  }

  connection_phase_++;
  if (connection_phase_ >= LightCount * 2)
  {
    connection_phase_ = 0;
  }

  last_connection_step_ms_ = now_ms;
  render();
}

void RearLights::renderConnectionAnimation()
{
  for (uint8_t i = 0; i < LightCount; i++)
  {
    setOutputPixel(i, OffColor);
  }

  uint8_t first_lit = 0;
  uint8_t last_lit = connection_phase_;
  if (connection_phase_ >= LightCount)
  {
    first_lit = connection_phase_ - LightCount + 1;
    last_lit = LightCount - 1;
  }

  for (uint8_t i = first_lit; i <= last_lit && i < LightCount; i++)
  {
    setOutputPixel(i, BluetoothBlue);
  }
}

void RearLights::advanceConnectedBlink(unsigned long now_ms)
{
  if (now_ms - last_connected_blink_step_ms_ < ConnectedBlinkMs)
  {
    return;
  }

  connected_blink_phase_++;
  last_connected_blink_step_ms_ = now_ms;

  if (connected_blink_phase_ >= 4)
  {
    connected_blink_active_ = false;
  }

  render();
}

void RearLights::renderConnectedBlink()
{
  bool blink_on = (connected_blink_phase_ % 2) == 0;

  for (uint8_t i = 0; i < LightCount; i++)
  {
    setOutputPixel(i, blink_on ? BluetoothBlue : OffColor);
  }
}

void RearLights::setOutputPixel(uint8_t index, RgbColor color)
{
  if (index >= LightCount)
  {
    return;
  }

  strip_.setPixelColor(index, toStripColor(color));
}

RgbColor RearLights::scaleColor(RgbColor color, uint8_t numerator, uint8_t denominator) const
{
  if (denominator == 0)
  {
    return color;
  }

  return {
      static_cast<uint8_t>((static_cast<uint16_t>(color.red) * numerator) / denominator),
      static_cast<uint8_t>((static_cast<uint16_t>(color.green) * numerator) / denominator),
      static_cast<uint8_t>((static_cast<uint16_t>(color.blue) * numerator) / denominator)};
}

uint32_t RearLights::toStripColor(RgbColor color) const
{
  return strip_.Color(color.red, color.green, color.blue);
}
