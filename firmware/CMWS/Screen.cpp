#include "Screen.h"

#include <Wire.h>
#include <U8g2lib.h>

namespace {

// Page-buffer mode (the "_1_"): the frame is sent in eight horizontal
// bands of 128 bytes instead of one 1024-byte buffer. The Blue Pill
// only has 20 KB of RAM.
// R2 = rotated 180 degrees, the panel is mounted upside down.
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2);

PanelOutputs s_lastDrawn;
bool         s_everDrawn   = false;
uint32_t     s_lastDrawMs  = 0;

bool    s_powerSaveForced = false;  // panel switch on OFF
bool    s_powerSaveActive = true;   // what the display is actually doing
uint8_t s_contrast        = 0;
bool    s_contrastKnown   = false;

void applyPowerSave(bool off) {
  if (off == s_powerSaveActive) {
    return;
  }
  s_powerSaveActive = off;
  u8g2.setPowerSave(off ? 1 : 0);

  // Coming back from power save, the panel must be redrawn even if the
  // model is unchanged.
  if (!off) {
    s_everDrawn = false;
  }
}

void applyContrast(uint8_t value) {
  if (s_contrastKnown && value == s_contrast) {
    return;
  }
  s_contrast      = value;
  s_contrastKnown = true;
  u8g2.setContrast(value);
}

// Writes an unsigned value as decimal into buf (max 5 digits + NUL).
// Hand-rolled on purpose: snprintf would drag the whole printf
// formatting machinery into a 64 KB flash budget.
void formatNumber(uint16_t value, char* buf) {
  char  digits[5];
  uint8_t n = 0;

  do {
    digits[n++] = static_cast<char>('0' + (value % 10));
    value /= 10;
  } while (value != 0 && n < sizeof(digits));

  for (uint8_t i = 0; i < n; i++) {
    buf[i] = digits[n - 1 - i];
  }
  buf[n] = '\0';
}

}  // namespace

namespace Screen {

void begin() {
  Wire.setSDA(cfg::PIN_OLED_SDA);
  Wire.setSCL(cfg::PIN_OLED_SCL);

  // begin() before setClock(): setClock dereferences a handle that only
  // begin() initialises.
  Wire.begin();
  Wire.setClock(cfg::I2C_CLOCK_HZ);

  u8g2.begin();

  s_powerSaveActive = false;
  u8g2.setPowerSave(0);
}

void setBrightness(uint16_t potRaw) {
  if (s_powerSaveForced) {
    return;
  }

  if (potRaw < cfg::SCREEN_OFF_BELOW) {
    applyPowerSave(true);
    return;
  }

  applyPowerSave(false);

  const uint32_t span = cfg::ADC_MAX - cfg::SCREEN_OFF_BELOW;
  applyContrast(static_cast<uint8_t>(
      ((potRaw - cfg::SCREEN_OFF_BELOW) * 255UL) / span));
}

void setPowerSave(bool off) {
  s_powerSaveForced = off;
  if (off) {
    applyPowerSave(true);
  }
}

void render(const PanelOutputs& out, uint32_t now) {
  if (s_powerSaveActive) {
    return;
  }

  if (s_everDrawn && out == s_lastDrawn) {
    return;  // nothing moved, do not spend 25 ms on I2C
  }

  if (s_everDrawn &&
      static_cast<uint32_t>(now - s_lastDrawMs) < cfg::SCREEN_MIN_PERIOD_MS) {
    return;
  }

  char flares[6];
  char chaff[6];
  formatNumber(out.flares, flares);
  formatNumber(out.chaff, chaff);

  u8g2.firstPage();
  do {
    // Everything inside this block runs once per band - drawing only,
    // no measurement, no computation.
    u8g2.setFont(u8g2_font_helvB18_tf);

    u8g2.drawStr(50, 27, "F");
    u8g2.drawStr(50, 59, "C");

    u8g2.drawStr(90, 27, flares);
    u8g2.drawStr(90, 59, chaff);
  } while (u8g2.nextPage());

  s_lastDrawn  = out;
  s_everDrawn  = true;
  s_lastDrawMs = now;
}

}  // namespace Screen
