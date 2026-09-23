#pragma once

#include <Arduino.h>

#include "Config.h"

// =====================================================================
// LED driver - software PWM
//
// Hardware PWM is not an option on this panel. On the F103C8 the six
// LED pins are TIM1 CH1/CH2/CH3 (PA8/PA9/PA10) and the complementary
// outputs CH1N/CH2N/CH3N (PB13/PB14/PB15) of those same three
// channels. A channel and its complement share one compare register,
// so at best three independent duty cycles - with the PB pins forced
// to the inverse of the PA pins. No other timer reaches PB13/14/15 on
// a 48-pin package.
//
// So: one timer interrupt, six LEDs, independent brightness each.
// =====================================================================

namespace Leds {

// Configures the pins and starts the PWM timer.
void begin();

// Per-LED brightness, 0..cfg::BRIGHTNESS_MAX percent.
// Out-of-range indexes are ignored, levels are clamped.
void set(uint8_t index, uint8_t percent);

// Convenience for a bitfield: bit i lit means LED i at full brightness.
void setMask(uint8_t mask);

// Global scale applied on top of every per-LED level, 0..100 percent.
// This is what the brightness pot drives, and what "panel off" uses.
void setMaster(uint8_t percent);

}  // namespace Leds
