#pragma once

#include <Arduino.h>

#include "PanelState.h"

// =====================================================================
// OLED screen - SH1106 128x64 over I2C
//
// Owns the U8g2 instance and the layout. Everything it draws comes
// from a PanelOutputs, so the display never reads the hardware and the
// game will later be able to feed it without touching this file.
//
// Redraws are both rate-limited and skipped when nothing changed: a
// full frame is ~25 ms of blocking I2C, and the panel is static most
// of the time.
// =====================================================================

namespace Screen {

void begin();

// Applies the brightness policy of the panel from a raw pot reading:
// below ~5 % the screen is turned off, above that the contrast ramps
// from 0 to 255. Commands are only sent when the value really changed.
void setBrightness(uint16_t potRaw);

// Forces the screen off regardless of the pot (panel switch on OFF).
void setPowerSave(bool off);

// Draws the panel if the rate limit allows it and the model changed.
void render(const PanelOutputs& out, uint32_t now);

}  // namespace Screen
