#pragma once

#include <Arduino.h>

// =====================================================================
// USB HID joystick device class.
//
// The STM32duino core only ships a keyboard+mouse HID class and a CDC
// class, with descriptors that cannot be overridden. This file is a
// self-contained HID class registered on the same USB device stack:
// one interface, one interrupt IN endpoint, a fixed 8-byte report.
//
// Build requirement: Tools > USB support = "HID (keyboard and mouse)"
// (fqbn option usb=HID). That menu is what makes the core compile its
// USB stack at all. Never include <Keyboard.h> or <Mouse.h>: their
// begin() would re-initialise the single USB peripheral behind our
// back.
// =====================================================================

namespace UsbHidJoystick {

// 11 buttons padded to 16 bits (bit 0 = button 1), then X and Y as
// little-endian 16-bit values in 0..4095. Fixed by the report descriptor.
constexpr uint8_t REPORT_SIZE = 6;

// Brings the USB peripheral up and starts enumeration. Non-blocking
// apart from the 10 ms D+ pulse the core uses to force the host to
// re-enumerate.
void begin();

// Housekeeping, to call on every loop iteration: shuts the stack down
// after the interrupt handler has flagged a reset storm, and brings it
// back up after cfg::USB_RETRY_MS. Cheap when nothing is wrong.
void service(uint32_t now);

// True once the host has configured the device. Nothing can be sent
// before that.
bool configured();

// Queues one report on the interrupt endpoint. Returns false, without
// touching anything, when the device is not configured yet or the
// previous report has not been picked up by the host.
bool send(const uint8_t* report);

}  // namespace UsbHidJoystick
