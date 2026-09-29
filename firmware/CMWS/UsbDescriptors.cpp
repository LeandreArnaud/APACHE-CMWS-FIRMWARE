#include "UsbDescriptors.h"

#include "Config.h"

namespace {

constexpr uint8_t lo(uint16_t v) { return static_cast<uint8_t>(v & 0xFF); }
constexpr uint8_t hi(uint16_t v) { return static_cast<uint8_t>(v >> 8); }

static_assert(2 + 2 * (sizeof(cfg::USB_MANUFACTURER_NAME) - 1) <= UsbDesc::kStringMax,
              "manufacturer string too long");
static_assert(2 + 2 * (sizeof(cfg::USB_PRODUCT_NAME) - 1) <= UsbDesc::kStringMax,
              "product string too long");

uint16_t asciiToStringDesc(const char* ascii, uint8_t* out, uint16_t cap) {
  uint16_t n = 2;
  for (const char* p = ascii; *p != '\0' && n + 2 <= cap; p++) {
    out[n++] = static_cast<uint8_t>(*p);
    out[n++] = 0;
  }
  out[0] = static_cast<uint8_t>(n);
  out[1] = 0x03;   // STRING descriptor
  return n;
}

void writeHexUnicode(uint32_t value, uint8_t* out, uint8_t digits) {
  for (uint8_t i = 0; i < digits; i++) {
    const uint8_t nibble = static_cast<uint8_t>(value >> 28);
    out[2 * i]     = static_cast<uint8_t>(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
    out[2 * i + 1] = 0;
    value <<= 4;
  }
}

}  // namespace

namespace UsbDesc {

const uint8_t device[18] = {
  0x12,                    // bLength
  0x01,                    // bDescriptorType: DEVICE
  0x00, 0x02,              // bcdUSB 2.00
  0x00, 0x00, 0x00,        // class/subclass/protocol: defined per interface
  64,                      // bMaxPacketSize0
  lo(cfg::USB_VID), hi(cfg::USB_VID),
  lo(cfg::USB_PID), hi(cfg::USB_PID),
  0x00, 0x01,              // bcdDevice 1.00
  0x01,                    // iManufacturer
  0x02,                    // iProduct
  0x03,                    // iSerialNumber
  0x01                     // bNumConfigurations
};

uint8_t config[34] = {
  // Configuration
  0x09, 0x02,
  34, 0x00,                // wTotalLength
  0x01,                    // bNumInterfaces
  0x01,                    // bConfigurationValue
  0x00,                    // iConfiguration
  0x80,                    // bmAttributes: bus powered
  0x32,                    // bMaxPower: 100 mA
  // Interface
  0x09, 0x04,
  0x00,                    // bInterfaceNumber
  0x00,                    // bAlternateSetting
  0x01,                    // bNumEndpoints
  0x03,                    // bInterfaceClass: HID
  0x00,                    // bInterfaceSubClass: no boot protocol
  0x00,                    // bInterfaceProtocol: none
  0x00,                    // iInterface
  // HID
  kHidDescSize, kHidDescriptorType,
  0x11, 0x01,              // bcdHID 1.11
  0x00,                    // bCountryCode
  0x01,                    // bNumDescriptors
  kHidReportDescType,
  lo(kReportDescSize), hi(kReportDescSize),
  // Endpoint
  0x07, 0x05,
  kEpInAddr,
  0x03,                    // interrupt
  kEpInSize, 0x00,
  kEpIntervalMs
};

static_assert(cfg::USB_BUTTON_COUNT == 11, "report descriptor hard-codes 11 buttons + 5 padding bits");

const uint8_t report[44] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x04,        // Usage (Joystick)
  0xA1, 0x01,        // Collection (Application)
  0x05, 0x09,        //   Usage Page (Button)
  0x19, 0x01,        //   Usage Minimum (Button 1)
  0x29, 0x0B,        //   Usage Maximum (Button 11)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0x01,        //   Logical Maximum (1)
  0x75, 0x01,        //   Report Size (1)
  0x95, 0x0B,        //   Report Count (11)
  0x81, 0x02,        //   Input (Data, Var, Abs)      -> bits 0..10
  0x95, 0x05,        //   Report Count (5)
  0x81, 0x03,        //   Input (Const, Var, Abs)     -> padding to 16 bits
  0x05, 0x01,        //   Usage Page (Generic Desktop)
  0x09, 0x30,        //   Usage (X)
  0x09, 0x31,        //   Usage (Y)
  0x15, 0x00,        //   Logical Minimum (0)
  0x26, 0xFF, 0x0F,  //   Logical Maximum (4095)
  0x75, 0x10,        //   Report Size (16)
  0x95, 0x02,        //   Report Count (2)
  0x81, 0x02,        //   Input (Data, Var, Abs)      -> bytes 2..5
  0xC0               // End Collection
};
static_assert(sizeof(report) == kReportDescSize, "report descriptor length drifted");

const uint8_t langId[4] = { 0x04, 0x03, 0x09, 0x04 };   // English (US)

uint8_t qualifier[10] = { 0x0A, 0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 64, 0x01, 0x00 };

uint16_t manufacturerString(uint8_t* out, uint16_t cap) {
  return asciiToStringDesc(cfg::USB_MANUFACTURER_NAME, out, cap);
}

uint16_t productString(uint8_t* out, uint16_t cap) {
  return asciiToStringDesc(cfg::USB_PRODUCT_NAME, out, cap);
}

uint16_t serialString(uint8_t* out, uint16_t cap) {
  constexpr uint16_t kLen = 2 + 2 * 12;
  if (cap < kLen) {
    return 0;
  }
  out[0] = kLen;
  out[1] = 0x03;
  writeHexUnicode(HAL_GetUIDw0() + HAL_GetUIDw2(), &out[2], 8);
  writeHexUnicode(HAL_GetUIDw1(), &out[18], 4);
  return kLen;
}

}  // namespace UsbDesc
