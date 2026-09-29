#pragma once

#include <Arduino.h>

#include "Config.h"

// =====================================================================
// Panel state
//
// The seam of the firmware. Two structs describe two opposite data
// flows that the legacy sketch mixed together:
//
//   switches / pots  --> PanelInputs   --> [ later: Link -> game ]
//   screen / LEDs    <-- PanelOutputs  <-- [ later: Link <- game ]
//
// Today PanelOutputs just holds the constants the legacy sketch drew
// inline, and PanelInputs has almost no consumer. That is expected:
// PanelInputs is what will be sent to the game.
// =====================================================================

// Position of the OFF / ON / TEST rotary.
//
// Unknown is not a defensive extra: a real rotary passes through a dead
// zone between detents where no contact is closed. The legacy sketch
// silently treated that as "on".
enum class ModeSwitch : uint8_t {
  Off,
  On,
  Test,
  Unknown
};

// What the panel measures.
struct PanelInputs {
  uint8_t    switches        = 0;   // bit i = switch i is closed (debounced)
  uint8_t    switchesChanged = 0;   // bit i = switch i changed on this poll
  ModeSwitch mode            = ModeSwitch::Unknown;
  bool       modeChanged     = false;
  uint16_t   potBrightness   = 0;   // LAMP pot, smoothed, 0..4095
  uint16_t   potAux          = 0;   // AUDIO pot, smoothed, 0..4095
};

// What the panel displays.
struct PanelOutputs {
  uint16_t flares  = 60;    // the "F" number
  uint16_t chaff   = 30;    // the "C" number
  uint8_t  ledMask = 0x3F;  // bit i = LED i should be lit (all six, as in v0.1)
};

inline bool operator==(const PanelOutputs& a, const PanelOutputs& b) {
  return a.flares == b.flares && a.chaff == b.chaff && a.ledMask == b.ledMask;
}

inline bool operator!=(const PanelOutputs& a, const PanelOutputs& b) {
  return !(a == b);
}
