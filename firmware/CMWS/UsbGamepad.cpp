#include "UsbGamepad.h"

#include <string.h>

#include "Config.h"
#include "UsbHidJoystick.h"

namespace {

constexpr uint8_t kReportSize = UsbHidJoystick::REPORT_SIZE;

// Button bits, 0-based (bit n = button n+1): levers first, rotary last.
constexpr uint8_t kRotaryOffBit  = 2 * cfg::LEVER_COUNT;
constexpr uint8_t kRotaryOnBit   = kRotaryOffBit + 1;
constexpr uint8_t kRotaryTestBit = kRotaryOffBit + 2;

static_assert(cfg::USB_BUTTON_COUNT <= 16, "button field is 16 bits wide");
static_assert(kReportSize == 2 + 2 + 2, "report = 16 button bits + X + Y");

uint8_t  s_lastSent[kReportSize] = { 0 };
uint32_t s_lastSentAt = 0;
bool     s_everSent   = false;

uint16_t buttonBits(const PanelInputs& in) {
  uint16_t bits = 0;

  for (uint8_t l = 0; l < cfg::LEVER_COUNT; l++) {
    const cfg::Lever& lever = cfg::LEVERS[l];
    if (in.switches & (1u << lever.contactA)) bits |= 1u << (2 * l);
    if (in.switches & (1u << lever.contactB)) bits |= 1u << (2 * l + 1);
  }

  switch (in.mode) {
    case ModeSwitch::Off:     bits |= 1u << kRotaryOffBit;  break;
    case ModeSwitch::On:      bits |= 1u << kRotaryOnBit;   break;
    case ModeSwitch::Test:    bits |= 1u << kRotaryTestBit; break;
    case ModeSwitch::Unknown: break;
  }

  return bits;
}

uint16_t clampAxis(uint16_t raw) {
  return raw > cfg::ADC_MAX ? cfg::ADC_MAX : raw;
}

void buildReport(const PanelInputs& in, uint8_t* out) {
  const uint16_t buttons = buttonBits(in);
  const uint16_t x = clampAxis(in.potBrightness);
  const uint16_t y = clampAxis(in.potAux);

  out[0] = static_cast<uint8_t>(buttons);
  out[1] = static_cast<uint8_t>(buttons >> 8);
  out[2] = static_cast<uint8_t>(x);
  out[3] = static_cast<uint8_t>(x >> 8);
  out[4] = static_cast<uint8_t>(y);
  out[5] = static_cast<uint8_t>(y >> 8);
}

}  // namespace

namespace UsbGamepad {

void begin() {
  UsbHidJoystick::begin();
}

void update(const PanelInputs& in, uint32_t now) {
  UsbHidJoystick::service(now);

  uint8_t report[kReportSize];
  buildReport(in, report);

  const bool changed   = !s_everSent || memcmp(report, s_lastSent, kReportSize) != 0;
  const bool heartbeat = static_cast<uint32_t>(now - s_lastSentAt) >= cfg::USB_HEARTBEAT_MS;

  if (!changed && !heartbeat) {
    return;
  }

  // A refused send (not configured, previous report still in flight)
  // leaves the last-sent copy alone, so the change is retried on the
  // next iteration instead of being lost.
  if (UsbHidJoystick::send(report)) {
    memcpy(s_lastSent, report, kReportSize);
    s_lastSentAt = now;
    s_everSent   = true;
  }
}

}  // namespace UsbGamepad
