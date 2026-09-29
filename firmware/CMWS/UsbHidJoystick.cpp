// =====================================================================
// USB HID joystick, backend A: the core's HAL-based USB device stack.
//
// Compiled only when Tools > USB support = "HID (keyboard and mouse)"
// (USBCON + USBD_USE_HID_COMPOSITE). With "None", UsbBareMetal.cpp
// provides the same API on top of the raw peripheral instead.
// =====================================================================

#include "UsbHidJoystick.h"

#if defined(USBCON) && defined(USBD_USE_HID_COMPOSITE)

#include "Config.h"
#include "UsbDescriptors.h"

// Pulls in usbd_conf.h, usbd_def.h, usbd_ioreq.h, usbd_ctlreq.h and the
// HAL PCD driver. All of them carry extern "C" guards.
#include "usbd_core.h"

#include <string.h>

// Everything in this file is static on purpose. The core still compiles
// its own keyboard+mouse class and descriptors (they are discarded by
// the linker), so any global sharing a name with them - USBD_Desc,
// USBD_StrDesc, hUSBD_Device_HID, USBD_COMPOSITE_HID, ... - would be a
// duplicate-symbol link error.
namespace {

// HID request codes. Copied rather than taken from usbd_hid_composite.h,
// which drags in a header without C++ guards.
constexpr uint8_t kHidReqGetReport    = 0x01;
constexpr uint8_t kHidReqGetIdle      = 0x02;
constexpr uint8_t kHidReqGetProtocol  = 0x03;
constexpr uint8_t kHidReqSetIdle      = 0x0A;
constexpr uint8_t kHidReqSetProtocol  = 0x0B;
constexpr uint8_t kHidReportTypeInput = 0x01;

constexpr uint8_t kEpInAddr  = UsbDesc::kEpInAddr;
constexpr uint8_t kEpInIndex = kEpInAddr & 0x0F;
constexpr uint8_t kEpInSize  = UsbDesc::kEpInSize;

// Packet memory layout (local byte addresses in the 512-byte PMA).
//
// The core lays it out for its keyboard+mouse class: buffer table
// 0..23, EP0 OUT 24..87, EP0 IN 88..151 - and then, by a bug in its
// arithmetic, the two HID buffers at 92 and 100, inside EP0 IN.
//
// We override all three. Two reasons:
//  - our endpoint must not share EP0's buffer;
//  - on the Hangshun HK32F103 (non-A) found on some Blue Pills, local
//    addresses 0..63 are not RAM at all: they are registers implementing
//    the buffer descriptor table (ADDRx 16-bit, COUNTx_TX 10-bit,
//    COUNTx_RX with a hardware-owned count field). A packet buffer placed
//    there silently loses its data - the core's EP0 OUT at 24 is exactly
//    that case, and enumeration dies on the first SETUP. Measured with a
//    memory probe, 2026-09-27. Genuine STM32F103 parts do not care where
//    the buffers are, so this layout costs nothing there.
constexpr uint16_t kEp0OutPmaAddress = 0x40;   // 64 bytes: 0x40..0x7F
constexpr uint16_t kEp0InPmaAddress  = 0x80;   // 64 bytes: 0x80..0xBF
constexpr uint16_t kEpInPmaAddress   = 0xC0;   // 8 bytes:  0xC0..0xC7

uint8_t s_stringScratch[UsbDesc::kStringMax];

// --- Descriptor callbacks -----------------------------------------

uint8_t* getDeviceDesc(USBD_SpeedTypeDef speed, uint16_t* length) {
  (void)speed;
  *length = sizeof(UsbDesc::device);
  return const_cast<uint8_t*>(UsbDesc::device);
}

uint8_t* getLangIdDesc(USBD_SpeedTypeDef speed, uint16_t* length) {
  (void)speed;
  *length = sizeof(UsbDesc::langId);
  return const_cast<uint8_t*>(UsbDesc::langId);
}

uint8_t* getManufacturerDesc(USBD_SpeedTypeDef speed, uint16_t* length) {
  (void)speed;
  *length = UsbDesc::manufacturerString(s_stringScratch, sizeof(s_stringScratch));
  return s_stringScratch;
}

uint8_t* getProductDesc(USBD_SpeedTypeDef speed, uint16_t* length) {
  (void)speed;
  *length = UsbDesc::productString(s_stringScratch, sizeof(s_stringScratch));
  return s_stringScratch;
}

uint8_t* getSerialDesc(USBD_SpeedTypeDef speed, uint16_t* length) {
  (void)speed;
  *length = UsbDesc::serialString(s_stringScratch, sizeof(s_stringScratch));
  return s_stringScratch;
}

// The configuration and interface string indexes are 0, so the core
// never asks for them (and it NULL-checks before calling anyway).
USBD_DescriptorsTypeDef s_descriptors = {
  getDeviceDesc,
  getLangIdDesc,
  getManufacturerDesc,
  getProductDesc,
  getSerialDesc,
  nullptr,   // GetConfigurationStrDescriptor
  nullptr    // GetInterfaceStrDescriptor
#if (USBD_CLASS_USER_STRING_DESC == 1)
  , nullptr  // GetUserStrDescriptor (slot enabled by the core's usbd_conf.h)
#endif
};

// -------------------------------------------------------------------
// Class state
//
// Kept in a plain static struct rather than behind pClassData: the core
// can deliver interface requests before it has called our Init (the
// device is only ADDRESSED then), and the reference class answers
// those with a bare failure that leaves EP0 hanging.
// -------------------------------------------------------------------
struct State {
  uint8_t       protocol   = 0;
  uint8_t       idleRate   = 0;
  uint8_t       altSetting = 0;
  volatile bool busy       = false;   // report in flight on the IN endpoint
};
State s_state;

// The one and only device handle in the program.
USBD_HandleTypeDef s_dev;

// Last report handed to the hardware. Also what GET_REPORT returns.
uint8_t s_report[UsbHidJoystick::REPORT_SIZE] = { 0 };

// Reset-storm guard state. The counter lives in the interrupt, the
// recovery runs from loop(), see service().
volatile uint16_t s_resetCount     = 0;
volatile uint32_t s_resetWindowAt  = 0;
volatile bool     s_stormDetected  = false;
bool              s_down           = false;
uint32_t          s_retryAt        = 0;

// Runs inside the USB interrupt, once per bus reset. Past the threshold
// it masks every USB interrupt source in place: the handler then
// returns and never fires again, which is what hands the CPU back to
// setup()/loop(). The stack is torn down properly from service().
void noteBusReset() {
  const uint32_t now = HAL_GetTick();
  if (static_cast<uint32_t>(now - s_resetWindowAt) > cfg::USB_STORM_WINDOW_MS) {
    s_resetWindowAt = now;
    s_resetCount    = 0;
  }
  if (++s_resetCount >= cfg::USB_STORM_RESETS) {
    USB->CNTR       = static_cast<uint16_t>(USB->CNTR & 0x00FFu);  // high byte = all interrupt masks
    s_stormDetected = true;
  }
}

// --- Class callbacks ----------------------------------------------

uint8_t classInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx) {
  (void)cfgidx;
  pdev->pClassDataCmsit[pdev->classId] = &s_state;
  pdev->pClassData = &s_state;

  // Full speed only on this chip: no high-speed interval to pick.
  pdev->ep_in[kEpInIndex].bInterval = UsbDesc::kEpIntervalMs;
  (void)USBD_LL_OpenEP(pdev, kEpInAddr, USBD_EP_TYPE_INTR, kEpInSize);
  pdev->ep_in[kEpInIndex].is_used = 1;

  s_state.busy = false;
  return USBD_OK;
}

// Also reached on bus reset and disconnect, which is exactly when a
// report that will never be acknowledged must stop counting as busy.
uint8_t classDeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx) {
  (void)cfgidx;
  (void)USBD_LL_CloseEP(pdev, kEpInAddr);
  pdev->ep_in[kEpInIndex].is_used   = 0;
  pdev->ep_in[kEpInIndex].bInterval = 0;
  pdev->pClassDataCmsit[pdev->classId] = nullptr;

  s_state.busy = false;
  noteBusReset();
  return USBD_OK;
}

uint8_t classSetup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req) {
  static uint16_t statusInfo = 0;
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest) {
        // For the SET_ requests the core sends the status stage itself
        // once we return OK; we only record the value.
        case kHidReqSetProtocol:
          s_state.protocol = static_cast<uint8_t>(req->wValue);
          break;
        case kHidReqGetProtocol:
          (void)USBD_CtlSendData(pdev, &s_state.protocol, 1);
          break;
        case kHidReqSetIdle:
          s_state.idleRate = static_cast<uint8_t>(req->wValue >> 8);
          break;
        case kHidReqGetIdle:
          (void)USBD_CtlSendData(pdev, &s_state.idleRate, 1);
          break;
        case kHidReqGetReport:
          if ((req->wValue >> 8) == kHidReportTypeInput) {
            (void)USBD_CtlSendData(pdev, s_report,
                                   MIN(UsbHidJoystick::REPORT_SIZE, req->wLength));
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;
        default:
          // SET_REPORT included: it carries an OUT data stage we never
          // arm, so a stall is the correct answer.
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest) {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            (void)USBD_CtlSendData(pdev, reinterpret_cast<uint8_t*>(&statusInfo), 2);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR: {
          uint8_t* buf = nullptr;
          uint16_t len = 0;
          const uint8_t type = static_cast<uint8_t>(req->wValue >> 8);
          if (type == UsbDesc::kHidReportDescType) {
            buf = const_cast<uint8_t*>(UsbDesc::report);
            len = MIN(UsbDesc::kReportDescSize, req->wLength);
          } else if (type == UsbDesc::kHidDescriptorType) {
            buf = &UsbDesc::config[UsbDesc::kConfigHidOffset];
            len = MIN(UsbDesc::kHidDescSize, req->wLength);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          (void)USBD_CtlSendData(pdev, buf, len);
          break;
        }

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            (void)USBD_CtlSendData(pdev, &s_state.altSetting, 1);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            s_state.altSetting = static_cast<uint8_t>(req->wValue);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          // The core has already cleared the stall and sent the status
          // stage before handing this to us: touching EP0 here would
          // break the transaction. If it was our endpoint, whatever was
          // in flight is gone.
          if (LOBYTE(req->wIndex) == kEpInAddr) {
            s_state.busy = false;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:   // vendor requests
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return static_cast<uint8_t>(ret);
}

// Runs in the USB interrupt once the host has read the report.
uint8_t classDataIn(USBD_HandleTypeDef* pdev, uint8_t epnum) {
  (void)pdev;
  if (epnum == kEpInIndex) {
    s_state.busy = false;
  }
  return USBD_OK;
}

uint8_t* getConfigDesc(uint16_t* length) {
  *length = sizeof(UsbDesc::config);
  return UsbDesc::config;
}

uint8_t* getQualifierDesc(uint16_t* length) {
  *length = sizeof(UsbDesc::qualifier);
  return UsbDesc::qualifier;
}

// The core calls Init, DeInit, Setup and GetFSConfigDescriptor without
// checking for NULL; the other slots it checks. The high-speed entries
// are never reached on a full-speed-only chip but cost nothing.
USBD_ClassTypeDef s_class = {
  classInit,
  classDeInit,
  classSetup,
  nullptr,           // EP0_TxSent
  nullptr,           // EP0_RxReady
  classDataIn,
  nullptr,           // DataOut
  nullptr,           // SOF
  nullptr,           // IsoINIncomplete
  nullptr,           // IsoOUTIncomplete
  getConfigDesc,     // GetHSConfigDescriptor
  getConfigDesc,     // GetFSConfigDescriptor
  getConfigDesc,     // GetOtherSpeedConfigDescriptor
  getQualifierDesc   // GetDeviceQualifierDescriptor
};

}  // namespace

namespace UsbHidJoystick {

void begin() {
  s_resetCount    = 0;
  s_resetWindowAt = HAL_GetTick();
  s_stormDetected = false;
  s_down          = false;

  // USBD_Init reaches the core's USBD_LL_Init: it pulses D+ low for
  // 10 ms so the host re-enumerates, initialises the PCD, and lays out
  // packet memory from its fixed keyboard+mouse table. Our buffers are
  // then moved before anything opens an endpoint. Keep these calls back
  // to back: D+ is already released after the first one.
  (void)USBD_Init(&s_dev, &s_descriptors, 0);
  PCD_HandleTypeDef* pcd = static_cast<PCD_HandleTypeDef*>(s_dev.pData);
  (void)HAL_PCDEx_PMAConfig(pcd, 0x00,      PCD_SNG_BUF, kEp0OutPmaAddress);
  (void)HAL_PCDEx_PMAConfig(pcd, 0x80,      PCD_SNG_BUF, kEp0InPmaAddress);
  (void)HAL_PCDEx_PMAConfig(pcd, kEpInAddr, PCD_SNG_BUF, kEpInPmaAddress);
  (void)USBD_RegisterClass(&s_dev, &s_class);

  // Clone workaround, measured on a Blue Pill whose F103 is not genuine:
  // its USB block keeps the RESET flag asserted for as long as the
  // function-enable bit (DADDR.EF) is 0. Genuine silicon only raises
  // RESET on a real bus reset. Left alone, that stale flag fires the
  // instant USBD_Start() unmasks interrupts - while HAL_PCD_Start()
  // still holds the HAL lock - so the handler's HAL_PCD_SetAddress()
  // returns HAL_BUSY without ever setting EF, the flag never drops, and
  // the interrupt re-enters forever (the storm guard above exists for
  // exactly that case). Enabling the function and clearing the flag
  // first costs nothing on genuine parts: they end up in the same
  // state a bus reset would put them in anyway.
  USB->DADDR = static_cast<uint16_t>(USB_DADDR_EF);
  USB->ISTR  = static_cast<uint16_t>(~USB_ISTR_RESET);

  (void)USBD_Start(&s_dev);
}

void service(uint32_t now) {
  if (s_stormDetected) {
    // Interrupts are already masked, so this runs undisturbed. Stop and
    // DeInit both call our DeInit again; the counters are cleared
    // afterwards so those calls do not count as resets.
    (void)USBD_Stop(&s_dev);
    (void)USBD_DeInit(&s_dev);
    s_resetCount    = 0;
    s_stormDetected = false;
    s_down          = true;
    s_retryAt       = now + cfg::USB_RETRY_MS;
    return;
  }

  if (s_down && static_cast<int32_t>(now - s_retryAt) >= 0) {
    begin();   // clears s_down; a persisting storm trips the guard again
  }
}

bool configured() {
  return !s_down && s_dev.dev_state == USBD_STATE_CONFIGURED;
}

bool send(const uint8_t* report) {
  // No lock needed: the interrupt only ever clears busy, and we only
  // set it after seeing it clear.
  if (!configured() || s_state.busy) {
    return false;
  }
  memcpy(s_report, report, REPORT_SIZE);
  s_state.busy = true;
  (void)USBD_LL_Transmit(&s_dev, kEpInAddr, s_report, REPORT_SIZE);
  return true;
}

}  // namespace UsbHidJoystick

#endif  // USBCON && USBD_USE_HID_COMPOSITE
