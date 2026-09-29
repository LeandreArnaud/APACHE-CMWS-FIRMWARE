#pragma once

#include <Arduino.h>

// =====================================================================
// USB descriptors of the joystick, shared by both USB backends
// (the core's HAL stack and the bare-metal stack).
//
// One HID interface, Usage Page Generic Desktop / Usage Joystick, one
// interrupt IN endpoint 0x81. Report: 11 buttons padded to 16 bits, then
// X + Y (16-bit, 0..4095): 6 bytes. No report IDs.
// =====================================================================

namespace UsbDesc {

constexpr uint8_t  kEpInAddr     = 0x81;
constexpr uint8_t  kEpInSize     = 8;
constexpr uint8_t  kEpIntervalMs = 10;

constexpr uint8_t  kHidDescriptorType = 0x21;
constexpr uint8_t  kHidReportDescType = 0x22;

extern const uint8_t device[18];

// Not const: the core's stack patches byte 1 in place when it serves it
// as an "other speed" configuration.
extern uint8_t config[34];
constexpr uint8_t kConfigHidOffset = 9 + 9;   // the 9-byte HID descriptor inside config
constexpr uint8_t kHidDescSize     = 9;

extern const uint8_t report[44];
constexpr uint16_t kReportDescSize = 44;

extern const uint8_t langId[4];
extern uint8_t qualifier[10];

// String descriptors, built on demand into `out` (UTF-16LE, with the
// 2-byte header). `cap` is the buffer size; the return value is the
// descriptor length actually written.
uint16_t manufacturerString(uint8_t* out, uint16_t cap);
uint16_t productString(uint8_t* out, uint16_t cap);
uint16_t serialString(uint8_t* out, uint16_t cap);   // from the chip UID, 12 hex digits

constexpr uint16_t kStringMax = 64;   // enough for every string above (static_assert in .cpp)

}  // namespace UsbDesc
