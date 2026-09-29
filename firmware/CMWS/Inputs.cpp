#include "Inputs.h"

namespace {

// -------------------------------------------------------------------
// One debounced active-low pin.
//
// The raw reading has to disagree with the stable state for
// cfg::DEBOUNCE_TICKS consecutive polls before the state flips. Any
// single poll that agrees resets the counter, so a bouncing contact
// never gets through. One counter per pin, so one noisy switch cannot
// delay the others.
// -------------------------------------------------------------------
struct DebouncedPin {
  uint8_t counter = 0;
  bool    stable  = false;

  // Returns true when the stable state changed on this call.
  bool update(uint32_t pin) {
    const bool raw = (digitalRead(pin) == LOW);  // active low

    if (raw == stable) {
      counter = 0;
      return false;
    }

    if (++counter < cfg::DEBOUNCE_TICKS) {
      return false;
    }

    stable  = raw;
    counter = 0;
    return true;
  }
};

// -------------------------------------------------------------------
// One smoothed analog input.
//
// Fixed-point exponential moving average: the accumulator holds the
// value shifted left by POT_EMA_SHIFT, which is what makes it converge
// exactly instead of stalling once the difference gets smaller than the
// shift. On top of that, a new value is only published once it has
// moved further than POT_DEADBAND - otherwise the few LSB of ADC noise
// would shimmer the LEDs and redraw the screen forever.
// -------------------------------------------------------------------
struct SmoothedPot {
  uint32_t accumulator = 0;
  uint16_t published   = 0;

  void prime(uint32_t pin) {
    const uint16_t raw = analogRead(pin);
    accumulator = static_cast<uint32_t>(raw) << cfg::POT_EMA_SHIFT;
    published   = raw;
  }

  void update(uint32_t pin) {
    accumulator += analogRead(pin);
    accumulator -= accumulator >> cfg::POT_EMA_SHIFT;

    const uint16_t filtered = accumulator >> cfg::POT_EMA_SHIFT;
    const int32_t  delta    = static_cast<int32_t>(filtered) - published;
    const bool     atEnd    = (filtered == 0) || (filtered >= cfg::ADC_MAX);

    if (atEnd || abs(delta) > cfg::POT_DEADBAND) {
      published = filtered;
    }
  }
};

PanelInputs  s_state;
DebouncedPin s_switch[cfg::SWITCH_COUNT];
DebouncedPin s_mode[3];   // Off, On, Test - same order as kModePin
SmoothedPot  s_potBrightness;
SmoothedPot  s_potAux;
uint32_t     s_lastPollMs = 0;

const uint32_t kModePin[3] = {
  cfg::PIN_MODE_OFF, cfg::PIN_MODE_ON, cfg::PIN_MODE_TEST
};

// Decodes the three debounced mode contacts. Exactly one closed is a
// valid position; zero (between detents) or two (a shorting contact
// mid-travel) is Unknown.
ModeSwitch decodeMode() {
  uint8_t closedCount = 0;
  uint8_t closedIndex = 0;

  for (uint8_t i = 0; i < 3; i++) {
    if (s_mode[i].stable) {
      closedCount++;
      closedIndex = i;
    }
  }

  if (closedCount != 1) {
    return ModeSwitch::Unknown;
  }

  switch (closedIndex) {
    case 0:  return ModeSwitch::Off;
    case 1:  return ModeSwitch::On;
    default: return ModeSwitch::Test;
  }
}

}  // namespace

namespace Inputs {

void begin() {
  analogReadResolution(12);

  for (uint8_t i = 0; i < cfg::SWITCH_COUNT; i++) {
    pinMode(cfg::PIN_SWITCH[i], INPUT_PULLUP);
  }

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(kModePin[i], INPUT_PULLUP);
  }

  pinMode(cfg::PIN_POT_BRIGHTNESS, INPUT_ANALOG);
  pinMode(cfg::PIN_POT_AUX, INPUT_ANALOG);

  // Prime every filter so nothing ramps up from zero after power-on.
  for (uint8_t i = 0; i < cfg::SWITCH_COUNT; i++) {
    s_switch[i].stable = (digitalRead(cfg::PIN_SWITCH[i]) == LOW);
    if (s_switch[i].stable) {
      s_state.switches |= (1u << i);
    }
  }

  for (uint8_t i = 0; i < 3; i++) {
    s_mode[i].stable = (digitalRead(kModePin[i]) == LOW);
  }
  s_state.mode = decodeMode();

  s_potBrightness.prime(cfg::PIN_POT_BRIGHTNESS);
  s_potAux.prime(cfg::PIN_POT_AUX);

  s_state.potBrightness = s_potBrightness.published;
  s_state.potAux        = s_potAux.published;

  s_lastPollMs = millis();
}

void poll(uint32_t now) {
  // Clear last poll's edges first: an edge must be reported exactly
  // once, on the iteration that follows the sample that produced it.
  s_state.switchesChanged = 0;
  s_state.modeChanged     = false;

  if (static_cast<uint32_t>(now - s_lastPollMs) < cfg::INPUT_PERIOD_MS) {
    return;
  }
  s_lastPollMs = now;

  for (uint8_t i = 0; i < cfg::SWITCH_COUNT; i++) {
    if (s_switch[i].update(cfg::PIN_SWITCH[i])) {
      s_state.switchesChanged |= (1u << i);

      if (s_switch[i].stable) {
        s_state.switches |= (1u << i);
      } else {
        s_state.switches &= ~(1u << i);
      }
    }
  }

  bool modeContactMoved = false;
  for (uint8_t i = 0; i < 3; i++) {
    modeContactMoved |= s_mode[i].update(kModePin[i]);
  }

  if (modeContactMoved) {
    const ModeSwitch decoded = decodeMode();
    if (decoded != s_state.mode) {
      s_state.mode        = decoded;
      s_state.modeChanged = true;
    }
  }

  s_potBrightness.update(cfg::PIN_POT_BRIGHTNESS);
  s_potAux.update(cfg::PIN_POT_AUX);

  s_state.potBrightness = s_potBrightness.published;
  s_state.potAux        = s_potAux.published;
}

const PanelInputs& state() {
  return s_state;
}

}  // namespace Inputs
